"""Headless contract tests for the native X11 fixture window geometry."""

from pathlib import Path
import sys
import unittest

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/"scripts"))

from x11_vision_monitor import bbox_rectangle  # noqa: E402


class NativeX11VisionMonitorTests(unittest.TestCase):
    def test_pixel_bbox_maps_to_scaled_x11_rectangle(self):
        self.assertEqual(bbox_rectangle([100, 80, 240, 300], 0.5),
                         (50, 40, 70, 110))

    def test_one_pixel_bbox_remains_visible(self):
        self.assertEqual(bbox_rectangle([639, 479, 640, 480], 0.4),
                         (256, 192, 1, 1))

    def test_empty_bbox_is_not_promoted_to_a_visible_shape(self):
        self.assertEqual(bbox_rectangle([12, 20, 12, 20], 1.0),
                         (12, 20, 1, 1))


if __name__ == "__main__":
    unittest.main()
