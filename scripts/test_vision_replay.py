"""ROS-free tests for paired image/result replay fixtures."""

import json
from pathlib import Path
import sys
import tempfile
import unittest

WORKSPACE = Path(__file__).resolve().parents[1]
PKG = WORKSPACE/"linorobot2_vision"
sys.path.insert(0, str(PKG/"python"))

from linorobot2_vision import VisionReplayError, load_replay_case  # noqa: E402


class VisionReplayTests(unittest.TestCase):
    def manifest(self):
        return PKG/"fixtures"/"replay_manifest.json"

    def test_detection_case_pairs_svg_pixels_with_capture_and_result(self):
        case = load_replay_case(self.manifest(), "detection")
        self.assertEqual(case.display_name, "Synthetic person-like object")
        self.assertEqual((case.image_width, case.image_height), (640, 480))
        self.assertTrue(case.image_path.is_file())
        self.assertEqual(case.response["status"], "ok")
        self.assertEqual(case.response["result"]["capture"]["width"], case.image_width)
        self.assertGreater(len(case.shapes), 5)
        self.assertTrue(any(shape["kind"] == "rect" and shape["fill"] == "#fb7185"
                            for shape in case.shapes))

    def test_empty_result_is_replayed_with_the_same_scene(self):
        case = load_replay_case(self.manifest(), "empty")
        self.assertEqual(case.response["result"]["detections"], [])
        self.assertEqual((case.image_width, case.image_height), (640, 480))

    def test_unavailable_case_does_not_fabricate_an_image(self):
        case = load_replay_case(self.manifest(), "unavailable")
        self.assertEqual(case.response["status"], "unavailable")
        self.assertIsNone(case.image_path)
        self.assertEqual(case.shapes, ())

    def test_image_dimensions_must_match_capture_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            svg = root/"scene.svg"
            svg.write_text('<svg width="10" height="10"><rect width="10" height="10" fill="#07111f"/></svg>', encoding="utf-8")
            (root/"detection.json").write_text(
                (PKG/"fixtures"/"detection.json").read_text(encoding="utf-8"),
                encoding="utf-8")
            manifest = root/"manifest.json"
            manifest.write_text(json.dumps({"schema_version": "1.0", "cases": {
                "bad": {"display_name": "bad", "image": "scene.svg",
                         "response": "detection.json"}}}), encoding="utf-8")
            with self.assertRaisesRegex(VisionReplayError, "dimensions do not match"):
                load_replay_case(manifest, "bad")

    def test_svg_rejects_external_references_and_unknown_elements(self):
        with tempfile.TemporaryDirectory() as directory:
            svg = Path(directory)/"bad.svg"
            svg.write_text('<!DOCTYPE svg><svg width="1" height="1"/>', encoding="utf-8")
            from linorobot2_vision import load_svg_scene
            with self.assertRaisesRegex(VisionReplayError, "DTD/entity"):
                load_svg_scene(svg)
            svg.write_text('<svg width="1" height="1"><script/></svg>', encoding="utf-8")
            with self.assertRaisesRegex(VisionReplayError, "unsupported SVG element"):
                load_svg_scene(svg)


if __name__ == "__main__":
    unittest.main()
