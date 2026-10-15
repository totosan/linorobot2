"""ROS-free helper and policy tests for the bounded Phase 3 SIL probe."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location(
    "phase3_under_test", Path(__file__).with_name("verify_phase3.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def pose(x=0.0, y=0.0, yaw=0.0):
    return {"position": (x, y, 0.0),
            "orientation": (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))}


class ProgressTests(unittest.TestCase):
    def test_straight_progress_projects_along_current_heading(self):
        start = pose(0.0, 0.0, math.pi/2)
        end = pose(0.0, 0.12, math.pi/2)
        self.assertAlmostEqual(probe.odom_progress("straight", start, end), 0.12)
        self.assertAlmostEqual(probe.odom_progress("straight", start, pose(0.0, 0.12, math.pi/2), -1), -0.12)

    def test_turn_progress_handles_angle_wraparound_and_direction(self):
        start = {"yaw": math.pi-0.1}
        end = {"yaw": -math.pi+0.2}
        self.assertAlmostEqual(probe.odom_progress("turn", start, end), 0.3)
        self.assertAlmostEqual(probe.odom_progress("turn", end, start), -0.3)

    def test_unknown_segment_kind_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "unknown"):
            probe.odom_progress("curve", {}, {})

    def test_world_and_both_odom_deltas_match_in_same_frames(self):
        start, end = pose(0, 0, 0), pose(0.1, 0.2, math.pi/2)
        odom_start = {"position": (0, 0, 0), "yaw": 0.0}
        odom_end = {"position": (0.1, 0.2, 0), "yaw": math.pi/2}
        error = probe.compare_world_odom(start, end, odom_start, odom_end)
        self.assertAlmostEqual(error["translation_m"], 0.0)
        self.assertAlmostEqual(error["yaw_rad"], 0.0)
        odom_end["position"] = (0.2, 0.2, 0)
        self.assertAlmostEqual(probe.compare_world_odom(start, end, odom_start, odom_end)["translation_m"], 0.1)

    def test_imu_yaw_integral_uses_sim_timestamps_and_orders_samples(self):
        samples = [{"stamp_ns": 1_000_000_000, "angular_z": 0.4},
                   {"stamp_ns": 0, "angular_z": 0.2}]
        self.assertAlmostEqual(probe.integrate_imu_yaw(samples), 0.3)
        self.assertEqual(probe.integrate_imu_yaw(samples[:1]), 0.0)

    def test_axle_pose_is_translated_to_chassis_frame_for_truth_comparison(self):
        axle = {"position": (0.0, 0.0, 0.0), "yaw": math.pi/2,
                "orientation": pose(yaw=math.pi/2)["orientation"],
                "child_frame_id": "base_drive"}
        chassis = probe.drive_pose_as_footprint(axle)
        self.assertAlmostEqual(chassis["position"][0], 0.0, places=9)
        self.assertAlmostEqual(chassis["position"][1], 0.07, places=9)
        self.assertEqual(chassis["child_frame_id"], "base_footprint")

    def test_world_control_requests_are_explicit_pause_and_resume(self):
        pause = probe.world_control_command(True)
        resume = probe.world_control_command(False)
        self.assertIn("/world/fortress_test/control", pause)
        self.assertEqual(pause[-1], "pause: true")
        self.assertEqual(resume[-1], "pause: false")
        self.assertIn("ignition.msgs.WorldControl", pause)
        self.assertIn("ignition.msgs.Boolean", pause)


class TfOwnershipTests(unittest.TestCase):
    @staticmethod
    def node(names):
        return NS(get_publishers_info_by_topic=lambda topic:
                  [NS(node_name=name) for name in names])

    def test_ekf_is_the_unique_odom_tf_writer(self):
        report = {}
        probe.assert_tf_ownership(self.node(["ekf_filter_node", "robot_state_publisher"]), report)
        self.assertEqual(report["odom_tf_owner"], "ekf_filter_node")
        self.assertEqual(report["tf_publishers"], ["ekf_filter_node", "robot_state_publisher"])

    def test_duplicate_or_unexpected_tf_writer_is_rejected(self):
        for names in (["robot_state_publisher"],
                      ["ekf_filter_node", "ekf_filter_node", "robot_state_publisher"],
                      ["ekf_filter_node", "robot_state_publisher", "other_broadcaster"]):
            with self.subTest(names=names), self.assertRaises(RuntimeError):
                probe.assert_tf_ownership(self.node(names), {})


class SafetyScopeTests(unittest.TestCase):
    def test_reported_motion_bounds_stay_within_g2_limits(self):
        self.assertLessEqual(probe.LINEAR_LIMIT, 0.08)
        self.assertLessEqual(probe.ANGULAR_LIMIT, 0.30)
        self.assertGreater(probe.SIDE_METERS, 0)
        self.assertLess(probe.SIDE_METERS, 0.25)
        self.assertGreater(probe.TURN_RADIANS, 1.4)
        self.assertLess(probe.TURN_RADIANS, 1.7)


if __name__ == "__main__":
    unittest.main()
