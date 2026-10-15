"""ROS-free regression tests for the Phase-5 vision contract and fixture mock."""

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE = (Path(__file__).parents[1]/"linorobot2_vision")
sys.path.insert(0, str(PACKAGE/"python"))

from linorobot2_vision import (  # noqa: E402
    FixtureVisionBackend,
    VisionClockDomainError,
    VisionContractError,
    VisionFutureResultError,
    VisionNoResultError,
    VisionStaleResultError,
    load_response,
    normalize_jetson_detection_v1,
    require_fresh_result,
    validate_response,
)


class VisionContractTests(unittest.TestCase):
    def fixture(self, name):
        return PACKAGE/"fixtures"/name

    def source_fixture(self, name):
        return json.loads(self.fixture(name).read_text(encoding="utf-8"))

    @staticmethod
    def capture_context():
        return {"stamp_ns": 1_250_000_000, "clock_domain": "ros_sim",
                "frame_id": "camera_optical_frame", "width": 640, "height": 480}

    def test_jetson_v1_adapter_uses_capture_context_and_explicit_label_ids(self):
        source = self.source_fixture("jetson_v1_success.json")
        normalized = normalize_jetson_detection_v1(
            source, capture=self.capture_context(), model_version="configured-v1",
            labels_version="coco80-reviewed-v1", label_to_class_id={"person": 0})
        result = normalized["result"]
        self.assertEqual(result["capture"]["stamp_ns"], 1_250_000_000)
        self.assertEqual(result["capture"]["clock_domain"], "ros_sim")
        self.assertEqual(result["model"]["version"], "configured-v1")
        self.assertEqual(result["detections"][0]["class_id"], 0)
        self.assertEqual(result["detections"][0]["bbox_xyxy_px"], [100, 150, 300, 450])

    def test_jetson_v1_adapter_preserves_empty_success(self):
        normalized = normalize_jetson_detection_v1(
            self.source_fixture("jetson_v1_empty.json"),
            capture=self.capture_context(), model_version="configured-v1",
            labels_version="coco80-reviewed-v1", label_to_class_id={"person": 0})
        self.assertEqual(normalized["status"], "ok")
        self.assertEqual(normalized["result"]["detections"], [])

    def test_jetson_v1_adapter_separates_failure_and_rejects_unknown_labels(self):
        failed = normalize_jetson_detection_v1(
            self.source_fixture("jetson_v1_error.json"), capture={},
            model_version="configured-v1", labels_version="coco80-reviewed-v1",
            label_to_class_id={})
        self.assertEqual(failed["status"], "error")
        self.assertIsNone(failed["result"])
        with self.assertRaisesRegex(VisionNoResultError, "source_detection_failed"):
            require_fresh_result(failed, 1, "ros_sim", 1)

        with self.assertRaisesRegex(VisionContractError, "no reviewed class ID"):
            normalize_jetson_detection_v1(
                self.source_fixture("jetson_v1_success.json"),
                capture=self.capture_context(), model_version="configured-v1",
                labels_version="coco80-reviewed-v1", label_to_class_id={})

    def test_jetson_v1_adapter_never_uses_server_timestamp_as_capture_time(self):
        source = self.source_fixture("jetson_v1_success.json")
        capture = self.capture_context()
        normalized = normalize_jetson_detection_v1(
            source, capture=capture, model_version="configured-v1",
            labels_version="coco80-reviewed-v1", label_to_class_id={"person": 0})
        self.assertEqual(normalized["result"]["capture"], capture)
        source["timestampUTC"] = "invalid-and-ignored"
        normalized_again = normalize_jetson_detection_v1(
            source, capture=capture, model_version="configured-v1",
            labels_version="coco80-reviewed-v1", label_to_class_id={"person": 0})
        self.assertEqual(normalized_again["result"]["capture"], capture)

    def test_jetson_v1_adapter_rejects_missing_capture_context_and_invalid_bbox(self):
        source = self.source_fixture("jetson_v1_success.json")
        with self.assertRaisesRegex(VisionContractError, "capture context"):
            normalize_jetson_detection_v1(
                source, capture={}, model_version="configured-v1",
                labels_version="coco80-reviewed-v1", label_to_class_id={"person": 0})
        source["predictions"][0]["x_max"] = 700
        with self.assertRaisesRegex(VisionContractError, "image bounds"):
            normalize_jetson_detection_v1(
                source, capture=self.capture_context(), model_version="configured-v1",
                labels_version="coco80-reviewed-v1", label_to_class_id={"person": 0})

    def test_detection_fixture_is_valid_and_mock_returns_detached_responses(self):
        backend = FixtureVisionBackend(self.fixture("detection.json"))
        first = backend.get_response()
        second = backend.get_response()
        self.assertEqual(first["schema_version"], "1.0")
        self.assertEqual(first["result"]["detections"][0]["label"], "test_object")
        first["result"]["detections"].clear()
        self.assertEqual(len(second["result"]["detections"]), 1)

    def test_empty_detection_list_is_a_successful_observation(self):
        response = load_response(self.fixture("empty.json"))
        self.assertEqual(response["status"], "ok")
        self.assertEqual(response["result"]["detections"], [])
        self.assertEqual(require_fresh_result(
            response, 2_100_000_000, "ros_sim", 250_000_000), 100_000_000)

    def test_unavailable_is_valid_but_never_a_detection_result(self):
        response = load_response(self.fixture("unavailable.json"))
        self.assertEqual(response["status"], "unavailable")
        with self.assertRaisesRegex(VisionNoResultError, "backend_unavailable"):
            require_fresh_result(response, 3, "ros_sim", 1)

    def test_rejects_unknown_schema_and_extra_fields(self):
        response = load_response(self.fixture("detection.json"))
        changed = copy.deepcopy(response)
        changed["schema_version"] = "2.0"
        with self.assertRaisesRegex(VisionContractError, "unsupported schema_version"):
            validate_response(changed)
        changed = copy.deepcopy(response)
        changed["cmd_vel"] = {"linear_x": 1.0}
        with self.assertRaisesRegex(VisionContractError, "unknown=.*cmd_vel"):
            validate_response(changed)

    def test_rejects_invalid_capture_model_and_error_envelopes(self):
        original = load_response(self.fixture("detection.json"))
        mutations = (
            (lambda d: d["result"]["capture"].update(clock_domain="wall"), "clock_domain"),
            (lambda d: d["result"]["capture"].update(clock_domain=[]), "clock_domain"),
            (lambda d: d["result"]["capture"].update(width=True), "capture.width"),
            (lambda d: d["result"]["model"].update(version=" "), "model.version"),
        )
        for mutate, expected in mutations:
            changed = copy.deepcopy(original)
            mutate(changed)
            with self.subTest(expected=expected), self.assertRaisesRegex(VisionContractError, expected):
                validate_response(changed)

        bad_error = {"schema_version": "1.0", "status": "error", "result": None,
                     "error": {"code": "Bad Code", "message": "failed"}}
        with self.assertRaisesRegex(VisionContractError, "stable lowercase"):
            validate_response(bad_error)

    def test_rejects_invalid_detection_values_and_pixel_bounds(self):
        original = load_response(self.fixture("detection.json"))
        mutations = (
            (lambda d: d.update(class_id=True), "class_id"),
            (lambda d: d.update(confidence=1.01), "confidence"),
            (lambda d: d.update(confidence=float("nan")), "finite number"),
            (lambda d: d.update(confidence=10**1000), "finite number"),
            (lambda d: d.update(bbox_xyxy_px=[100, 80, 700, 300]), "image bounds"),
            (lambda d: d.update(bbox_xyxy_px=[100, 80, 100, 300]), "non-empty box"),
            (lambda d: d.update(track_id=" "), "track_id"),
        )
        for mutate, expected in mutations:
            changed = copy.deepcopy(original)
            mutate(changed["result"]["detections"][0])
            with self.subTest(expected=expected), self.assertRaisesRegex(VisionContractError, expected):
                validate_response(changed)

    def test_freshness_checks_stale_future_and_clock_domain(self):
        response = load_response(self.fixture("detection.json"))
        self.assertEqual(require_fresh_result(
            response, 1_500_000_000, "ros_sim", 500_000_000), 250_000_000)
        with self.assertRaises(VisionStaleResultError):
            require_fresh_result(response, 2_000_000_001, "ros_sim", 500_000_000)
        with self.assertRaises(VisionFutureResultError):
            require_fresh_result(response, 1_000_000_000, "ros_sim", 500_000_000)
        with self.assertRaises(VisionClockDomainError):
            require_fresh_result(response, 1_800_000_000, "unix_utc", 1_000_000_000)

    def test_fixture_loader_rejects_invalid_json_and_structures(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"bad.json"
            path.write_text("{not json", encoding="utf-8")
            with self.assertRaisesRegex(VisionContractError, "cannot load"):
                load_response(path)
            path.write_text(json.dumps({"schema_version": "1.0"}), encoding="utf-8")
            with self.assertRaisesRegex(VisionContractError, "keys mismatch"):
                load_response(path)


if __name__ == "__main__":
    unittest.main()