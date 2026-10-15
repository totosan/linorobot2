"""ROS-free tests for the RDP fixture replay rendering."""

from pathlib import Path
import sys
import unittest


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE/"linorobot2_vision"
sys.path.insert(0, str(PACKAGE/"python"))
sys.path.insert(0, str(WORKSPACE/"scripts"))

from linorobot2_vision import load_response  # noqa: E402
from vision_replay_monitor import (  # noqa: E402
    CANVAS_HEIGHT,
    CANVAS_WIDTH,
    render_dashboard,
    scene_rows,
)


class VisionReplayMonitorTests(unittest.TestCase):
    def fixture(self, name):
        return load_response(PACKAGE/"fixtures"/name)

    def test_detection_fixture_renders_box_and_capture_metadata(self):
        response = self.fixture("detection.json")
        dashboard = render_dashboard(response, "detection")
        self.assertIn("OFFLINE MOCK ONLY", dashboard)
        self.assertIn("camera_optical_frame | 640x480", dashboard)
        self.assertIn("test_object conf=0.930", dashboard)
        self.assertIn("box=[100, 80, 240, 300]", dashboard)
        self.assertTrue(any("+" in row and "-" in row for row in scene_rows(response)))

    def test_empty_fixture_is_visibly_distinct_from_unavailable(self):
        empty = render_dashboard(self.fixture("empty.json"), "empty")
        unavailable = render_dashboard(self.fixture("unavailable.json"), "unavailable")
        self.assertIn("Status: OK", empty)
        self.assertIn("valid, successful empty result", empty)
        self.assertIn("Status: UNAVAILABLE", unavailable)
        self.assertIn("backend_unavailable", unavailable)
        self.assertNotIn("Capture:", unavailable)

    def test_scene_dimensions_and_boxes_at_image_edges_are_safe(self):
        response = self.fixture("detection.json")
        detection = response["result"]["detections"][0]
        detection["bbox_xyxy_px"] = [630, 470, 640, 480]
        rows = scene_rows(response)
        self.assertEqual(len(rows), CANVAS_HEIGHT)
        self.assertTrue(all(len(row) == CANVAS_WIDTH for row in rows))
        self.assertEqual(rows[-1][-1], "+")

    def test_fixture_cycle_has_three_known_choices(self):
        from vision_replay_monitor import FIXTURES, load_fixture

        self.assertEqual(set(FIXTURES), {"1", "2", "3"})
        self.assertEqual(load_fixture("1")[0], "detection")
        self.assertEqual(load_fixture("2")[0], "empty")
        self.assertEqual(load_fixture("3")[0], "unavailable")


if __name__ == "__main__":
    unittest.main()
