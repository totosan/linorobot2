"""ROS-free tests for projected LaserScan/detection-box association."""

import copy
from pathlib import Path
import sys
import unittest

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / "linorobot2_vision" / "python"))

from linorobot2_vision.lidar_camera_fusion import (  # noqa: E402
    fuse_scan_with_detections,
    transform_point,
)


CAPTURE_NS = 1_000_000_000
OPTICAL_FRAME = "camera_optical_frame"
CAMERA_FROM_LASER = (0.0, -0.04, -0.12, 0.5, -0.5, 0.5, 0.5)
BASE_FROM_LASER = (0.02, 0.0, 0.085, 0.0, 0.0, 0.0, 1.0)


def fixture_response(box=(100.0, 80.0, 240.0, 300.0)):
    return {
        "schema_version": "1.0",
        "status": "ok",
        "result": {
            "capture": {
                "stamp_ns": CAPTURE_NS,
                "clock_domain": "ros_sim",
                "frame_id": OPTICAL_FRAME,
                "width": 640,
                "height": 480,
            },
            "model": {"name": "test-mock", "version": "1", "labels_version": "1"},
            "detections": [{
                "class_id": 1,
                "label": "test_object",
                "confidence": 0.9,
                "bbox_xyxy_px": list(box),
            }],
        },
        "error": None,
    }


def camera_info(stamp=CAPTURE_NS, frame=OPTICAL_FRAME):
    return {
        "stamp_ns": stamp,
        "frame_id": frame,
        "width": 640,
        "height": 480,
        "projection": [554.3827, 0.0, 320.0, 0.0,
                       0.0, 554.3827, 240.0, 0.0,
                       0.0, 0.0, 1.0, 0.0],
        "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def scan(ranges=(2.1, 2.1, 2.1), stamp=CAPTURE_NS + 10_000_000):
    return {
        "stamp_ns": stamp,
        "frame_id": "laser",
        "angle_min": 0.25,
        "angle_increment": 0.01,
        "range_min": 0.08,
        "range_max": 12.0,
        "ranges": list(ranges),
    }


class LidarCameraFusionTests(unittest.TestCase):
    def test_quaternion_transform_maps_body_axes_to_optical_axes(self):
        forward = transform_point((1.0, 0.0, 0.0),
                                  (0, 0, 0, 0.5, -0.5, 0.5, 0.5))
        left = transform_point((0.0, 1.0, 0.0),
                               (0, 0, 0, 0.5, -0.5, 0.5, 0.5))
        up = transform_point((0.0, 0.0, 1.0),
                             (0, 0, 0, 0.5, -0.5, 0.5, 0.5))
        for actual, expected in zip(forward, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected, places=7)
        for actual, expected in zip(left, (-1.0, 0.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=7)
        for actual, expected in zip(up, (0.0, -1.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=7)

    def test_associates_projected_cluster_and_reports_surface_point(self):
        result = fuse_scan_with_detections(
            fixture_response(), camera_info(), scan(),
            CAMERA_FROM_LASER, BASE_FROM_LASER)
        detection = result["detections"][0]
        self.assertEqual(result["state"], "ok")
        self.assertEqual(detection["lidar"]["status"], "matched")
        self.assertEqual(detection["lidar"]["cluster_points"], 3)
        self.assertAlmostEqual(detection["lidar"]["range_m"], 2.119334, places=5)
        self.assertGreater(detection["lidar"]["bearing_rad"], 0.0)
        self.assertEqual(result["position_semantics"],
                         "median_lidar_surface_return_in_scan_plane")

    def test_returns_unmatched_for_box_with_no_projected_scan_points(self):
        result = fuse_scan_with_detections(
            fixture_response((500.0, 350.0, 620.0, 470.0)), camera_info(), scan(),
            CAMERA_FROM_LASER, BASE_FROM_LASER)
        self.assertEqual(result["detections"][0]["lidar"], {
            "status": "unmatched", "candidate_points": 0})

    def test_rejects_scan_outside_capture_time_window(self):
        with self.assertRaisesRegex(ValueError, "allowed capture-time window"):
            fuse_scan_with_detections(
                fixture_response(), camera_info(),
                scan(stamp=CAPTURE_NS + 100_000_001),
                CAMERA_FROM_LASER, BASE_FROM_LASER)

    def test_rejects_nonzero_distortion_until_rectified(self):
        info = camera_info()
        info["distortion"][0] = 0.1
        with self.assertRaisesRegex(ValueError, "requires rectification"):
            fuse_scan_with_detections(
                fixture_response(), info, scan(),
                CAMERA_FROM_LASER, BASE_FROM_LASER)

    def test_rejects_camera_frame_or_dimension_mismatch(self):
        info = camera_info(frame="camera_link")
        with self.assertRaisesRegex(ValueError, "frame IDs do not match"):
            fuse_scan_with_detections(
                fixture_response(), info, scan(),
                CAMERA_FROM_LASER, BASE_FROM_LASER)
        info = camera_info()
        info["width"] = 320
        with self.assertRaisesRegex(ValueError, "dimensions do not match"):
            fuse_scan_with_detections(
                fixture_response(), info, scan(),
                CAMERA_FROM_LASER, BASE_FROM_LASER)

    def test_skips_nonfinite_and_out_of_range_lidar_returns(self):
        result = fuse_scan_with_detections(
            fixture_response(), camera_info(), scan((float("nan"), 0.01, float("inf"))),
            CAMERA_FROM_LASER, BASE_FROM_LASER)
        self.assertEqual(result["detections"][0]["lidar"]["status"], "unmatched")

    def test_empty_success_is_distinct_from_failed_capture(self):
        response = fixture_response()
        response["result"]["detections"] = []
        result = fuse_scan_with_detections(
            response, camera_info(), scan(), CAMERA_FROM_LASER, BASE_FROM_LASER)
        self.assertEqual(result["state"], "empty")
        self.assertEqual(result["detections"], [])
        failed = copy.deepcopy(response)
        failed["status"] = "error"
        failed["result"] = None
        failed["error"] = {"code": "detector_error", "message": "test error"}
        self.assertEqual(fuse_scan_with_detections(
            failed, camera_info(), scan(), CAMERA_FROM_LASER, BASE_FROM_LASER)["state"],
            "error")


if __name__ == "__main__":
    unittest.main()
