"""ROS-free tests for Vision contract metadata and ROS outcome adaptation."""

from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/"linorobot2_vision"
sys.path.insert(0, str(PACKAGE/"python"))

from linorobot2_vision import (  # noqa: E402
    FixtureVisionBackend,
    VisionContractError,
    VisionOffBackend,
)
from linorobot2_vision.ros_adapter import (  # noqa: E402
    bind_fixture_capture,
    capture_from_camera_pair,
    error_response,
    parse_label_map,
    response_for_outcome,
    status_payload,
)


def header(frame_id, stamp_ns):
    return NS(
        frame_id=frame_id,
        stamp=NS(sec=stamp_ns//1_000_000_000,
                 nanosec=stamp_ns % 1_000_000_000))


def camera_pair(image_frame="camera_link", info_frame="camera_link",
                image_stamp=10_000_000_000, info_stamp=10_020_000_000,
                image_size=(640, 480), info_size=(640, 480)):
    image = NS(width=image_size[0], height=image_size[1],
               header=header(image_frame, image_stamp))
    info = NS(width=info_size[0], height=info_size[1],
              header=header(info_frame, info_stamp))
    return image, info


class VisionRosAdapterTests(unittest.TestCase):
    def test_camera_pair_becomes_capture_using_image_stamp_and_domain(self):
        image, info = camera_pair()
        capture = capture_from_camera_pair(
            image, info, clock_domain="ros_sim", pair_slop_ns=25_000_000)
        self.assertEqual(capture, {
            "stamp_ns": 10_000_000_000,
            "clock_domain": "ros_sim",
            "frame_id": "camera_link",
            "width": 640,
            "height": 480,
        })

    def test_camera_pair_rejects_frame_dimensions_time_and_expected_frame(self):
        cases = (
            (camera_pair(info_frame="other"), {}, "frame IDs"),
            (camera_pair(info_size=(320, 240)), {}, "dimensions"),
            (camera_pair(info_stamp=10_100_000_001), {}, "timestamps"),
            (camera_pair(), {"expected_frame_id": "camera_optical_frame"}, "expected"),
            (camera_pair(image_frame=""), {}, "frame_id"),
        )
        for (image, info), options, expected in cases:
            with self.subTest(expected=expected), self.assertRaisesRegex(
                    VisionContractError, expected):
                capture_from_camera_pair(
                    image, info, clock_domain="ros_sim", pair_slop_ns=50_000_000,
                    **options)

    def test_label_map_is_explicit_and_strict(self):
        self.assertEqual(parse_label_map('{"0":"person","1":"test_object"}'),
                         {0: "person", 1: "test_object"})
        for raw in ("not-json", "[]", "{}", '{"-1":"bad"}', '{"1":" "}'):
            with self.subTest(raw=raw), self.assertRaises(VisionContractError):
                parse_label_map(raw)

    def test_mock_response_rebinds_live_capture_and_checks_class_label_mapping(self):
        fixture_path = PACKAGE/"fixtures/detection.json"
        response = FixtureVisionBackend(fixture_path).get_response()
        original = deepcopy(response)
        capture = {
            "stamp_ns": 10_000_000_000,
            "clock_domain": "ros_sim",
            "frame_id": "camera_link",
            "width": 640,
            "height": 480,
        }
        rebound = bind_fixture_capture(response, capture, {1: "test_object"})
        self.assertEqual(rebound["result"]["capture"], capture)
        self.assertEqual(rebound["result"]["detections"][0]["label"], "test_object")
        self.assertEqual(response, original)
        for mapping, expected in (({}, "no reviewed label"),
                                  ({1: "person"}, "does not match")):
            with self.subTest(mapping=mapping), self.assertRaisesRegex(
                    VisionContractError, expected):
                bind_fixture_capture(response, capture, mapping)

    def test_error_outcomes_become_contract_errors_and_status_stays_separate(self):
        response = FixtureVisionBackend(PACKAGE/"fixtures/empty.json").get_response()
        outcome = NS(state="stale", error_code="stale_result",
                     message="too old", age_ns=300, detections=())
        contract = response_for_outcome(response, outcome)
        self.assertEqual(contract["status"], "error")
        self.assertEqual(contract["error"]["code"], "stale_result")
        diagnostic = status_payload(outcome, {"frame_id": "camera_link"})
        self.assertEqual(diagnostic["state"], "stale")
        self.assertEqual(diagnostic["capture"]["frame_id"], "camera_link")
        self.assertNotIn("cmd_vel", diagnostic)

        unavailable = VisionOffBackend().get_response()
        off_outcome = NS(state="unavailable", error_code="vision_disabled",
                         message="disabled", age_ns=None, detections=())
        self.assertEqual(response_for_outcome(unavailable, off_outcome), unavailable)
        self.assertEqual(error_response("invalid_capture_metadata", "bad frame")["status"],
                         "error")

    def test_ros_adapter_has_no_robot_command_interface(self):
        source = (PACKAGE/"python/linorobot2_vision/ros_node.py").read_text()
        self.assertNotIn("cmd_vel", source)
        self.assertNotIn("geometry_msgs", source)


if __name__ == "__main__":
    unittest.main()
