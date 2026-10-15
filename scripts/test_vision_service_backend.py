"""ROS-free tests for the Vision HTTP service backend."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
from threading import Thread
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"linorobot2_vision"/"python"))

from linorobot2_vision.contract import VisionContractError  # noqa: E402
from linorobot2_vision.service_backend import VisionHTTPServiceBackend  # noqa: E402


CAPTURE = {"stamp_ns": 100, "clock_domain": "ros_sim", "frame_id": "camera_link",
           "width": 2, "height": 1}
IMAGE = NS(width=2, height=1, encoding="rgb8", step=6,
           data=bytes([10, 20, 30, 40, 50, 60]))


class _Handler(BaseHTTPRequestHandler):
    response_payload = {}
    last_content_type = None
    last_body = b""

    def do_POST(self):
        type(self).last_content_type = self.headers.get("Content-Type")
        length = int(self.headers.get("Content-Length", "0"))
        type(self).last_body = self.rfile.read(length)
        body = json.dumps(type(self).response_payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        pass


class VisionServiceBackendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        cls.thread = Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.endpoint = f"http://127.0.0.1:{cls.server.server_port}/v1/vision/detection"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def _backend(self, mapping=None):
        return VisionHTTPServiceBackend(
            self.endpoint, timeout_sec=1.0, min_confidence=0.3,
            model_version="not-reported-by-source",
            labels_version="not-configured-yet",
            class_id_to_label={} if mapping is None else mapping)

    def test_empty_success_becomes_contract_v1_with_original_capture(self):
        _Handler.response_payload = {
            "success": True, "moduleId": "ObjectDetectionYOLOv11",
            "predictions": [], "timestampUTC": "response-time-only"}
        result = self._backend().get_response(IMAGE, CAPTURE)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["result"]["capture"], CAPTURE)
        self.assertEqual(result["result"]["model"], {
            "name": "ObjectDetectionYOLOv11",
            "version": "not-reported-by-source",
            "labels_version": "not-configured-yet"})
        self.assertEqual(result["result"]["detections"], [])
        self.assertIn("boundary=", _Handler.last_content_type)
        self.assertIn(b'name="image"', _Handler.last_body)
        self.assertIn(b"P6\n2 1\n255\n", _Handler.last_body)
        self.assertNotIn("timestampUTC", result["result"]["capture"])

    def test_unverified_mode_preserves_unknown_and_rejects_unmapped_detections(self):
        backend = VisionHTTPServiceBackend(
            self.endpoint, timeout_sec=1.0, min_confidence=0.3,
            model_version="unverified", labels_version="unverified",
            class_id_to_label={})
        _Handler.response_payload = {
            "success": True, "moduleId": "ObjectDetectionYOLOv11",
            "predictions": []}
        result = backend.get_response(IMAGE, CAPTURE)
        self.assertEqual(result["result"]["model"]["version"], "unverified")
        self.assertEqual(result["result"]["model"]["labels_version"], "unverified")
        self.assertEqual(result["result"]["detections"], [])

        _Handler.response_payload["predictions"] = [{
            "label": "person", "confidence": 0.91,
            "x_min": 0, "y_min": 0, "x_max": 2, "y_max": 1,
        }]
        with self.assertRaisesRegex(VisionContractError, "no reviewed class ID"):
            backend.get_response(IMAGE, CAPTURE)

    def test_detection_requires_reviewed_label_to_class_mapping(self):
        _Handler.response_payload = {
            "success": True, "moduleId": "ObjectDetectionYOLOv11",
            "predictions": [{"label": "person", "confidence": 0.91,
                             "x_min": 0, "y_min": 0, "x_max": 2, "y_max": 1}]}
        result = self._backend({4: "person"}).get_response(IMAGE, CAPTURE)
        self.assertEqual(result["result"]["detections"][0]["class_id"], 4)
        self.assertEqual(result["result"]["detections"][0]["label"], "person")
        with self.assertRaisesRegex(VisionContractError, "no reviewed class ID"):
            self._backend().get_response(IMAGE, CAPTURE)

    def test_failed_response_is_normalized_and_unknown_model_metadata_rejected(self):
        _Handler.response_payload = {
            "success": False, "message": "inference unavailable", "predictions": []}
        result = self._backend().get_response(IMAGE, CAPTURE)
        self.assertEqual(result["status"], "error")
        self.assertEqual(result["error"]["code"], "source_detection_failed")
        with self.assertRaisesRegex(ValueError, "model_version"):
            VisionHTTPServiceBackend(
                self.endpoint, timeout_sec=1.0, min_confidence=0.3,
                model_version="", labels_version="labels-v1", class_id_to_label={})

    def test_endpoint_rejects_embedded_credentials(self):
        with self.assertRaisesRegex(ValueError, "without embedded credentials"):
            VisionHTTPServiceBackend(
                "http://user:secret@127.0.0.1:8082/detect", timeout_sec=1.0,
                min_confidence=0.3, model_version="v1", labels_version="l1",
                class_id_to_label={})


if __name__ == "__main__":
    unittest.main()
