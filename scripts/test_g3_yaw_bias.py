"""Regression tests for the G3 yaw-bias analysis and the mu discrimination hook."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "g3_yaw_bias_under_test", ROOT/"scripts/analyze_g3_yaw_bias.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def phase3_report(raw=1.8552, gt=1.5278, imu=1.5214):
    segment = {
        "name": "turn_1", "kind": "turn",
        "ground_truth_delta": gt, "unfiltered_odom_delta": raw,
        "imu": {"integrated_yaw_rad": imu},
        "mean_wheel_velocity_rad_s": {"left_wheel_joint": -0.559,
                                      "right_wheel_joint": 0.559},
    }
    return {"square_segments": [dict(segment, name=f"turn_{i}")
                                for i in range(1, 5)]}


def motion_report():
    def segment(name, sign):
        return {"name": name, "signed_motion": sign*0.198,
                "command": {"angular_z_rps": sign*0.3},
                "odom_motion": {"unfiltered": {"signed_yaw_delta_rad": sign*0.2403}}}
    return {"segments": [segment("turn_positive", 1), segment("turn_negative", -1)]}


class YawBiasAnalysisTests(unittest.TestCase):
    def test_turn_ratios_and_effective_separation(self):
        rows, summary = analysis.analyze_turns(phase3_report())
        self.assertEqual(len(rows), 4)
        self.assertAlmostEqual(rows[0]["raw_over_gt"], 1.8552/1.5278, places=5)
        self.assertAlmostEqual(
            rows[0]["effective_no_slip_separation_m"],
            1.8552/1.5278*0.126, places=5)
        self.assertTrue(summary["imu_matches_gt_all"])
        self.assertAlmostEqual(summary["wheel_radius_m"], 0.03375)

    def test_motion_report_direction_independence(self):
        rows, summary = analysis.analyze_motion_report(motion_report())
        self.assertEqual(summary["directions"], ["turn_positive", "turn_negative"])
        self.assertAlmostEqual(summary["ratio_spread"], 0.0, places=6)
        self.assertAlmostEqual(rows[0]["raw_over_gt"], 0.2403/0.198, places=5)

    def test_rejects_missing_or_inconsistent_segments(self):
        with self.assertRaisesRegex(ValueError, "no turn segments"):
            analysis.analyze_turns({"square_segments": []})
        with self.assertRaisesRegex(ValueError, "positive yaw"):
            analysis.analyze_turns(phase3_report(raw=-1.0))
        bad = motion_report()
        bad["segments"][1]["signed_motion"] = 0.2
        with self.assertRaisesRegex(ValueError, "sign or magnitude"):
            analysis.analyze_motion_report(bad)

    def test_g2_harness_wheel_mu_hook_is_optional_bounded_and_documented(self):
        text = (ROOT/"scripts/test_phase2_motion.sh").read_text()
        self.assertIn("G2_WHEEL_MU", text)
        self.assertIn("wheel_mu:=${G2_WHEEL_MU}", text)
        self.assertIn("0.0 < x <= 5.0", text)
        self.assertIn("hypothesis test only; product default stays 1.0", text)
        launch = text.index("ros2 launch linorobot2_gazebo fortress.launch.py")
        self.assertLess(text.index("G2_LAUNCH_ARGS=()"), launch)

    def test_g2_harness_has_bounded_step_and_caster_hooks(self):
        text = (ROOT/"scripts/test_phase2_motion.sh").read_text()
        self.assertIn('G2_PHYSICS_STEP" != "0.001"', text)
        self.assertIn('G2_PHYSICS_STEP" != "0.0005"', text)
        self.assertIn('G2_INCLUDE_CASTER" != "true"', text)
        self.assertIn('G2_INCLUDE_CASTER" != "false"', text)
        self.assertIn('include_caster:=${G2_INCLUDE_CASTER}', text)
        self.assertIn('G2_LAUNCH_ARGS+=("wheel_mu:=${G2_WHEEL_MU}")', text)
        self.assertIn(
            'fi\nif [[ -n "${G2_PHYSICS_STEP:-}" ]]', text)
        self.assertIn('G2_CASTER_MU" != "0.01"', text)
        self.assertIn('G2_CASTER_MU" != "0.0"', text)
        self.assertIn('caster_mu:=${G2_CASTER_MU}', text)

    def test_world_variant_is_bounded_hashed_and_non_overwriting(self):
        spec = importlib.util.spec_from_file_location(
            "g3_world_variant_under_test",
            ROOT/"scripts/make_g3_world_variant.py")
        variant = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(variant)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/"baseline.sdf"
            output = root/"variant.sdf"
            source.write_text(
                "<sdf><world><physics><max_step_size>0.001</max_step_size>"
                "</physics></world></sdf>")
            result = variant.create_variant(source, output, 0.0005)
            self.assertIn("<max_step_size>0.0005</max_step_size>",
                          output.read_text())
            self.assertEqual(result["selected_step_seconds"], 0.0005)
            self.assertEqual(len(result["source_sha256"]), 64)
            with self.assertRaisesRegex(ValueError, "overwrite"):
                variant.create_variant(source, output, 0.0005)
            with self.assertRaisesRegex(ValueError, "exactly"):
                variant.create_variant(source, root/"bad.sdf", 0.0007)

    def test_caster_default_stays_enabled(self):
        xacro = (ROOT/"linorobot2_description/urdf/robots/"
                 "2wd_fortress.urdf.xacro").read_text()
        launch = (ROOT/"linorobot2_gazebo/launch/fortress.launch.py").read_text()
        self.assertIn('<xacro:arg name="include_caster" default="true"/>', xacro)
        self.assertIn('<xacro:arg name="caster_mu" default="0.01"/>', xacro)
        self.assertIn('<xacro:if value="$(arg include_caster)">', xacro)
        self.assertIn("name='include_caster', default_value='true'", launch)
        self.assertIn("name='caster_mu', default_value='0.01'", launch)


if __name__ == "__main__":
    unittest.main()
