"""ROS-free tests for the .71-protocol ZeroMQ backend and sim detector server."""

import json
from pathlib import Path
import sys
from threading import Event, Thread
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"linorobot2_vision"/"python"))
sys.path.insert(0, str(ROOT/"linorobot2_vision"/"sim_detector"))

import zmq  # noqa: E402

from linorobot2_vision.adapters import normalize_zmq_detection_v1  # noqa: E402
from linorobot2_vision.contract import VisionContractError  # noqa: E402
from linorobot2_vision.sim_service_config import validate_zmq_config  # noqa: E402
from linorobot2_vision.zmq_backend import (  # noqa: E402
    VisionZmqBackend, coco80_label_map)
import server  # noqa: E402

CAPTURE = {"stamp_ns": 100, "clock_domain": "ros_sim",
           "frame_id": "camera_optical_frame", "width": 4, "height": 2}
IMAGE = NS(width=4, height=2, encoding="rgb8", step=12, data=bytes(range(24)))
LABELS = {0: "person", 56: "chair"}


class _FakeDetector:
    def __init__(self, detections=None, error=None):
        self.detections = detections or []
        self.error = error
        self.shapes = []

    def detect(self, image_bgr):
        self.shapes.append(image_bgr.shape)
        if self.error:
            raise self.error
        return self.detections


class NormalizeTests(unittest.TestCase):
    def _norm(self, payload, **kwargs):
        return normalize_zmq_detection_v1(
            payload, capture=CAPTURE, model_name="yolo12n", model_version="v",
            labels_version="l", label_to_class_id={"person": 0}, **kwargs)

    def test_maps_label_and_clamps_box(self):
        response = self._norm({"detections": [
            {"box": [-1.5, 0.2, 5.0, 2.7], "label": "person", "confidence": 0.9}]})
        detection = response["result"]["detections"][0]
        self.assertEqual(detection["class_id"], 0)
        self.assertEqual(detection["bbox_xyxy_px"], [0.0, 0.2, 4.0, 2.0])
        self.assertEqual(response["result"]["capture"], CAPTURE)

    def test_error_payload_becomes_error_response(self):
        response = self._norm({"error": "Failed to decode image"})
        self.assertEqual(response["status"], "error")
        self.assertEqual(response["error"]["code"], "source_detection_failed")

    def test_empty_is_ok_and_min_confidence_filters(self):
        self.assertEqual(self._norm({"detections": []})["result"]["detections"], [])
        low = {"detections": [{"box": [0, 0, 1, 1], "label": "person",
                               "confidence": 0.2}]}
        self.assertEqual(self._norm(low, min_confidence=0.3)["result"]["detections"], [])

    def test_rejects_bad_payloads(self):
        good = {"box": [0, 0, 1, 1], "label": "person", "confidence": 0.5}
        for payload in (
                [], {}, {"detections": "x"},
                {"detections": [dict(good, label="dog")]},
                {"detections": [dict(good, box=[0, 0, 1])]},
                {"detections": [dict(good, box=[0, 0, float("nan"), 1])]},
                {"detections": [dict(good, box=[5, 0, 6, 1])]},
                {"detections": [dict(good, confidence=1.5)]},
                {"detections": [dict(good, confidence=True)]}):
            with self.subTest(payload=payload):
                with self.assertRaises(VisionContractError):
                    self._norm(payload)


class ProbeReportTests(unittest.TestCase):
    def test_expected_label_gate(self):
        sys.path.insert(0, str(ROOT/"scripts"))
        from probe_zmq_vision_service import build_report
        response = normalize_zmq_detection_v1(
            {"detections": []}, capture=CAPTURE, model_name="m", model_version="v",
            labels_version="l", label_to_class_id={"person": 0})
        self.assertEqual(build_report(response, "", 1.0)["status"], "PASS")
        failed = build_report(response, "person", 1.0)
        self.assertEqual(failed["status"], "FAIL")
        error = normalize_zmq_detection_v1(
            {"error": "x"}, capture=CAPTURE, model_name="m", model_version="v",
            labels_version="l", label_to_class_id={})
        self.assertEqual(build_report(error, "", 1.0)["status"], "FAIL")


class ConfigTests(unittest.TestCase):
    ARGS = ("v1", "l1", '{"0":"person"}')

    def test_profiles(self):
        self.assertTrue(validate_zmq_config(
            "sim_local", "tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556",
            *self.ARGS, ros_domain_id="42"))
        self.assertTrue(validate_zmq_config(
            "sim_local_isolated",
            "tcp://host.docker.internal:15555",
            "tcp://host.docker.internal:15556",
            *self.ARGS, ros_domain_id="42"))
        self.assertTrue(validate_zmq_config(
            "hardware_71", "tcp://192.168.0.71:5555", "tcp://192.168.0.71:5556",
            *self.ARGS, ros_domain_id="44"))

    def test_rejects_cross_profile_wrong_port_domain_and_metadata(self):
        cases = [
            ("sim_local", "tcp://192.168.0.71:5555", "tcp://192.168.0.71:5556", "42", self.ARGS),
            ("hardware_71", "tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556", "44", self.ARGS),
            ("sim_local", "tcp://127.0.0.1:5556", "tcp://127.0.0.1:5555", "42", self.ARGS),
            ("sim_local", "tcp://127.0.0.1:5555", "tcp://localhost:5556", "42", self.ARGS),
            ("sim_local_isolated", "tcp://127.0.0.1:15555", "tcp://127.0.0.1:15556", "42", self.ARGS),
            ("sim_local_isolated", "tcp://host.docker.internal:5555", "tcp://host.docker.internal:5556", "42", self.ARGS),
            ("hardware_71", "tcp://192.168.0.71:5555", "tcp://192.168.0.71:5556", "42", self.ARGS),
            ("sim_local", "tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556", "42", ("", "l", '{"0":"person"}')),
            ("sim_local", "tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556", "42", ("unverified", "l", '{"0":"person"}')),
            ("sim_local", "tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556", "42", ("v", "l", "{}")),
            ("other", "tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556", "42", self.ARGS),
        ]
        for profile, image, reply, domain, rest in cases:
            with self.subTest(profile=profile, image=image, domain=domain, rest=rest):
                with self.assertRaises(VisionContractError):
                    validate_zmq_config(profile, image, reply, *rest, ros_domain_id=domain)

    def test_coco_map_has_person_zero_and_80_labels(self):
        labels = coco80_label_map()
        self.assertEqual((labels[0], len(labels)), ("person", 80))


class LoopbackTests(unittest.TestCase):
    def _start(self, detector, pair_timeout_ms=5000):
        self.ctx = zmq.Context()
        self.stop = Event()
        # Keep both reservations open so the OS cannot return the same port.
        ports = []
        probes = []
        for _ in range(2):
            probe = self.ctx.socket(zmq.REP)
            probes.append(probe)
            ports.append(probe.bind_to_random_port("tcp://127.0.0.1"))
        for probe in probes:
            probe.close(linger=0)
        self.image_ep = f"tcp://127.0.0.1:{ports[0]}"
        self.reply_ep = f"tcp://127.0.0.1:{ports[1]}"
        self.thread = Thread(target=server.serve, args=(
            detector, self.stop, self.image_ep, self.reply_ep, pair_timeout_ms,
            self.ctx), daemon=True)
        self.thread.start()
        self.addCleanup(self._stop)

    def _stop(self):
        self.stop.set()
        self.thread.join(timeout=3)
        self.ctx.term()

    def _backend(self, **kwargs):
        args = dict(timeout_sec=2.0, min_confidence=0.3, model_name="yolo12n",
                    model_version="v", labels_version="l", class_id_to_label=LABELS,
                    context=self.ctx)
        args.update(kwargs)
        return VisionZmqBackend(self.image_ep, self.reply_ep, **args)

    def test_round_trip_detection(self):
        detector = _FakeDetector([{"box": [0.5, 0.0, 3.0, 2.0],
                                   "label": "person", "confidence": 0.8}])
        self._start(detector)
        response = self._backend().get_response(IMAGE, CAPTURE)
        self.assertEqual(response["status"], "ok")
        self.assertEqual(response["result"]["detections"][0]["class_id"], 0)
        self.assertEqual(detector.shapes, [(2, 4, 3)])

    def test_two_sequential_requests_stay_in_sync(self):
        self._start(_FakeDetector())
        backend = self._backend()
        for _ in range(2):
            self.assertEqual(backend.get_response(IMAGE, CAPTURE)["status"], "ok")

    def test_inference_failure_is_error_not_empty(self):
        self._start(_FakeDetector(error=RuntimeError("boom")))
        response = self._backend().get_response(IMAGE, CAPTURE)
        self.assertEqual(response["status"], "error")
        self.assertIn("boom", response["error"]["message"])

    def test_undecodable_image_reported(self):
        self.assertEqual(
            server.handle_request(_FakeDetector(), b"not an image", "detect"),
            {"error": "Failed to decode image"})
        self.assertIn("error", server.handle_request(_FakeDetector(), b"", "bogus"))

    def test_no_server_times_out_without_hanging(self):
        self.ctx = zmq.Context()
        self.addCleanup(self.ctx.term)
        backend = VisionZmqBackend(
            "tcp://127.0.0.1:1", "tcp://127.0.0.1:2", timeout_sec=0.2,
            min_confidence=0.3, model_name="m", model_version="v",
            labels_version="l", class_id_to_label=LABELS, context=self.ctx)
        with self.assertRaises(VisionContractError):
            backend.get_response(IMAGE, CAPTURE)

    def test_server_replies_only_json_detections_shape(self):
        self._start(_FakeDetector())
        push = self.ctx.socket(zmq.PUSH)
        req = self.ctx.socket(zmq.REQ)
        for sock in (push, req):
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.RCVTIMEO, 2000)
        push.connect(self.image_ep)
        req.connect(self.reply_ep)
        import cv2
        import numpy as np
        ok, png = cv2.imencode(".png", np.zeros((8, 8, 3), np.uint8))
        push.send(png.tobytes())
        req.send_string("detect")
        self.assertEqual(json.loads(req.recv()), {"detections": []})
        push.close()
        req.close()

    def test_constructor_validation(self):
        for bad in ({"timeout_sec": 0}, {"min_confidence": 2},
                    {"model_version": " "}, {"class_id_to_label": {}}):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    VisionZmqBackend(
                        "tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556",
                        **{**dict(timeout_sec=1, min_confidence=0.3,
                                  model_name="m", model_version="v",
                                  labels_version="l", class_id_to_label=LABELS),
                           **bad})
        with self.assertRaises(ValueError):
            VisionZmqBackend("http://x:1", "tcp://127.0.0.1:5556",
                             timeout_sec=1, min_confidence=0.3, model_name="m",
                             model_version="v", labels_version="l",
                             class_id_to_label=LABELS)


if __name__ == "__main__":
    unittest.main()
