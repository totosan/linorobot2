"""ROS-free tests for capture-matched pixel-box camera overlays."""

import copy
import json
from pathlib import Path
import sys
import unittest

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / "linorobot2_vision" / "python"))

from linorobot2_vision.image_overlay import boxes_for_capture, draw_bbox_overlay  # noqa: E402
from linorobot2_vision.contract import VisionContractError  # noqa: E402

FIXTURE = WORKSPACE / "linorobot2_vision" / "fixtures" / "detection.json"


def detection_response():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


class ImageOverlayTests(unittest.TestCase):
    def test_draws_rgb_box_border_and_preserves_interior(self):
        width, height, step = 6, 5, 18
        source = bytes([10, 20, 30] * width * height)
        result = draw_bbox_overlay(
            source, width=width, height=height, step=step, encoding="rgb8",
            boxes=[[1, 1, 5, 4]], thickness=1)

        def pixel(data, x, y):
            offset = y * step + x * 3
            return tuple(data[offset:offset + 3])

        self.assertEqual(pixel(result, 1, 1), (255, 32, 32))
        self.assertEqual(pixel(result, 4, 3), (255, 32, 32))
        self.assertEqual(pixel(result, 2, 2), (10, 20, 30))
        self.assertEqual(pixel(result, 0, 0), (10, 20, 30))
        self.assertEqual(pixel(source, 1, 1), (10, 20, 30))

    def test_supports_bgr_alpha_and_padded_rows(self):
        width, height, step = 3, 2, 14
        source = bytes([1] * (step * height))
        bgr = draw_bbox_overlay(
            source, width=width, height=height, step=step, encoding="bgr8",
            boxes=[[0, 0, 1, 1]], thickness=1)
        self.assertEqual(tuple(bgr[:3]), (32, 32, 255))
        self.assertEqual(bgr[width * 3:step], source[width * 3:step])

        rgba = draw_bbox_overlay(
            bytes([1] * (2 * 2 * 4)), width=2, height=2, step=8,
            encoding="rgba8", boxes=[[0, 0, 2, 2]], thickness=1)
        self.assertEqual(tuple(rgba[:4]), (255, 32, 32, 255))

    def test_rejects_invalid_image_layout_and_boxes(self):
        common = dict(width=4, height=3, step=12, encoding="rgb8", boxes=[])
        with self.assertRaisesRegex(ValueError, "unsupported image encoding"):
            draw_bbox_overlay(b"\0" * 36, **{**common, "encoding": "mono16"})
        with self.assertRaisesRegex(ValueError, "shorter"):
            draw_bbox_overlay(b"\0" * 35, **common)
        with self.assertRaisesRegex(ValueError, "positive width"):
            draw_bbox_overlay(b"\0" * 36, **{**common, "boxes": [[2, 1, 1, 2]]})

    def test_only_exact_capture_metadata_gets_a_box(self):
        response = detection_response()
        response["result"]["capture"] = {
            "stamp_ns": 2_000_000_000,
            "clock_domain": "ros_sim",
            "frame_id": "camera_link",
            "width": 640,
            "height": 480,
        }
        expected = [[100.0, 80.0, 240.0, 300.0]]
        self.assertEqual(boxes_for_capture(
            response, stamp_ns=2_000_000_000, frame_id="camera_link",
            width=640, height=480), expected)
        self.assertIsNone(boxes_for_capture(
            response, stamp_ns=2_000_000_001, frame_id="camera_link",
            width=640, height=480))
        self.assertIsNone(boxes_for_capture(
            response, stamp_ns=2_000_000_000, frame_id="other",
            width=640, height=480))

    def test_successful_empty_response_is_distinct_from_mismatch(self):
        response = {
            "schema_version": "1.0", "status": "ok",
            "result": {
                "capture": {
                    "stamp_ns": 5, "clock_domain": "ros_sim",
                    "frame_id": "camera_link", "width": 1, "height": 1,
                },
                "model": {"name": "mock", "version": "1", "labels_version": "1"},
                "detections": [],
            },
            "error": None,
        }
        self.assertEqual(boxes_for_capture(
            response, stamp_ns=5, frame_id="camera_link", width=1, height=1), [])
        invalid = copy.deepcopy(response)
        invalid["result"]["detections"] = [{"bbox_xyxy_px": [0, 0, 1, 1]}]
        with self.assertRaises(VisionContractError):
            boxes_for_capture(invalid, stamp_ns=5, frame_id="camera_link", width=1, height=1)


if __name__ == "__main__":
    unittest.main()
