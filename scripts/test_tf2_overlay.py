"""ROS-free regression tests for the explicit G4 dependency overlay guard."""

import importlib.util
from pathlib import Path
import tempfile
import unittest


spec = importlib.util.spec_from_file_location(
    "tf2_overlay_under_test", Path(__file__).with_name("verify_tf2_overlay.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class Tf2OverlayTests(unittest.TestCase):
    def test_default_overlay_root_prefers_canonical_then_legacy_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            home = Path(temporary)
            historical = home/"linobot2_tf2_fix"
            historical.mkdir()
            self.assertEqual(probe.default_overlay_root(home), historical)

            canonical = home/"linorobot2_tf2_fix"
            canonical.mkdir()
            self.assertEqual(probe.default_overlay_root(home), canonical)

    def output(self, prefix="/home/toto/linobot2_tf2_fix/install/lib"):
        return (f"  libtf2.so => {prefix}/libtf2.so (0x001)\n"
                f"  libtf2_ros.so => {prefix}/libtf2_ros.so (0x002)\n")

    def test_both_tf2_libraries_must_select_fix_overlay(self):
        selected = probe.validate_tf2_libraries(
            self.output(), Path("/home/toto/linobot2_tf2_fix"))
        self.assertEqual(set(selected), {"libtf2.so", "libtf2_ros.so"})

    def test_original_or_mixed_tf2_resolution_is_rejected(self):
        for output in (self.output("/opt/ros/humble/lib"),
                       self.output().replace(
                           "/home/toto/linobot2_tf2_fix/install/lib/libtf2_ros.so",
                           "/opt/ros/humble/lib/libtf2_ros.so")):
            with self.subTest(output=output), self.assertRaisesRegex(RuntimeError, "does not select"):
                probe.validate_tf2_libraries(output, Path("/home/toto/linobot2_tf2_fix"))

    def test_missing_vendor_dependency_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "unresolved"):
            probe.library_paths("  libddsc.so.0 => not found\n")

    def test_absent_tf2_library_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "absent"):
            probe.validate_tf2_libraries("", Path("/home/toto/linobot2_tf2_fix"))

    def test_g4_sources_fix_after_robot_overlay_before_any_launch(self):
        text = Path(__file__).with_name("test_phase4_gate.sh").read_text()
        self.assertIn("--tf2-fix)", text)
        robot = text.index("linorobot2_source_overlay")
        fix = text.index('source "$TF2_FIX_SETUP"')
        verify = text.index('scripts/verify_tf2_overlay.py')
        launch = text.index('ros2 launch')
        self.assertLess(robot, fix)
        self.assertLess(fix, verify)
        self.assertLess(verify, launch)
        self.assertIn('ENABLE_TF2_FIX="false"', text)


if __name__ == "__main__":
    unittest.main()