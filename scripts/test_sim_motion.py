"""ROS-free contract tests for G2 motion helpers: python3 -m unittest scripts/test_sim_motion.py."""

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest import mock


spec = importlib.util.spec_from_file_location(
    "sim_motion_under_test", Path(__file__).with_name("verify_sim_motion.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def pose_message(name="linorobot2_2wd", position=None, orientation=None):
    return json.dumps({"pose": [{"name": name,
                                  "position": position or {"x": 1.0, "y": 2.0, "z": 0.1},
                                  "orientation": orientation or {"w": 1.0}}]})


class PoseTests(unittest.TestCase):
    def test_single_and_multi_message_stream_uses_latest_pose(self):
        first = pose_message()
        later = pose_message(position={"x": 1.25, "y": 2.0, "z": 0.1},
                             orientation={"z": math.sin(0.2), "w": math.cos(0.2)})
        self.assertEqual(probe.parse_model_pose_stream(first)["position"], (1.0, 2.0, 0.1))
        result = probe.parse_model_pose_stream(first + "\n" + later)
        self.assertEqual(result["position"], (1.25, 2.0, 0.1))
        self.assertAlmostEqual(probe.yaw_from_quaternion(result["orientation"]), 0.4)

    def test_pose_parser_rejects_empty_duplicate_wrong_and_invalid_streams(self):
        for text in ("", "not json", "{}", '{"pose": []}',
                     pose_message(name="linorobot2_2wd::base_link"),
                     pose_message(position={"x": math.nan}),
                     pose_message(orientation={"w": 0}),
                     pose_message() + "\ninvalid", pose_message() + "\n{\"pose\": []}"):
            with self.subTest(text=text), self.assertRaises((TypeError, ValueError, json.JSONDecodeError)):
                probe.parse_model_pose_stream(text)

    def test_yaw_and_translation_helpers_handle_wrap_and_initial_heading(self):
        yaw = math.pi/2
        start = {"position": (1.0, 2.0, 0.0),
                 "orientation": (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))}
        end = {"position": (1.0, 2.1, 0.0), "orientation": start["orientation"]}
        self.assertAlmostEqual(probe.projected_translation(start, end), 0.1)
        self.assertAlmostEqual(probe.angle_delta(math.pi-0.1, -math.pi+0.1), 0.2)
        for q in ((0, 0, 0, 0), (0, 0, 0, 2), (math.nan, 0, 0, 1), (0, 0, 1)):
            with self.subTest(q=q), self.assertRaises(ValueError):
                probe.yaw_from_quaternion(q)

    def test_pose_reader_uses_bounded_command_and_latest_complete_message(self):
        output = pose_message() + "\n" + pose_message(position={"x": 2.0})
        result = NS(returncode=0, stdout=output, stderr="")
        with mock.patch.object(probe.time, "monotonic", return_value=10), \
                mock.patch.object(probe.subprocess, "run", return_value=result) as run:
            value = probe.read_model_pose(20)
        self.assertEqual(value["position"], (2.0, 0.0, 0.0))
        self.assertEqual(run.call_args.args[0], ["ign", "topic", "-e", "-t", probe.POSE_TOPIC,
                                                 "-n", "1", "--json-output"])
        self.assertEqual(run.call_args.kwargs["timeout"], 5)
        with mock.patch.object(probe.time, "monotonic", return_value=20), \
                self.assertRaisesRegex(RuntimeError, "deadline"):
            probe.read_model_pose(20)


class JointStateTests(unittest.TestCase):
    def checker(self):
        value = probe.MotionChecks.__new__(probe.MotionChecks)
        value.joint_samples, value.error = [], None
        return value

    def test_joint_mapping_is_name_based_and_values_must_be_finite(self):
        check = self.checker()
        check.on_joints(NS(name=["right_wheel_joint", "left_wheel_joint"],
                            position=[2.0, 1.0], velocity=[-3.0, 4.0]))
        self.assertEqual(check.joint_samples[0]["positions"],
                         {"left_wheel_joint": 1.0, "right_wheel_joint": 2.0})
        self.assertEqual(check.joint_samples[0]["velocities"],
                         {"left_wheel_joint": 4.0, "right_wheel_joint": -3.0})
        for message in (NS(name=["left_wheel_joint"], position=[0], velocity=[0]),
                        NS(name=list(probe.JOINTS), position=[0], velocity=[0, 0]),
                        NS(name=[*probe.JOINTS, probe.JOINTS[0]], position=[0, 0, 0], velocity=[0, 0, 0]),
                        NS(name=list(probe.JOINTS), position=[math.nan, 0], velocity=[0, 0])):
            with self.subTest(message=message):
                check = self.checker()
                check.on_joints(message)
                self.assertEqual(check.joint_samples, [])
                self.assertTrue(check.error)

    def test_clock_allows_startup_grace_then_rejects_absent_or_stopped_clock(self):
        check = probe.MotionChecks.__new__(probe.MotionChecks)
        check.error = None
        check.clock_ns = None
        check.clock_advances = 0
        check.clock_started_at = 10.0
        check.clock_progress_at = 10.0
        with mock.patch.object(probe.time, "monotonic", return_value=14.9):
            check.ensure_ok()
        with mock.patch.object(probe.time, "monotonic", return_value=15.1), \
                self.assertRaisesRegex(ValueError, "absent"):
            check.ensure_ok()
        check.clock_ns = 0
        check.clock_advances = 0
        with mock.patch.object(probe.time, "monotonic", return_value=14.9):
            check.ensure_ok()
        with mock.patch.object(probe.time, "monotonic", return_value=15.1), \
                self.assertRaisesRegex(ValueError, "not advancing"):
            check.ensure_ok()
        check.clock_advances = 1
        check.clock_progress_at = 10.0
        with mock.patch.object(probe.time, "monotonic", return_value=13.1), \
                self.assertRaisesRegex(ValueError, "stopped"):
            check.ensure_ok()

    def test_finite_vector_rejects_nan_and_infinity(self):
        self.assertEqual(probe.finite_vector([1, -2], "x"), (1.0, -2.0))
        for values in ([math.nan], [math.inf], []):
            with self.subTest(values=values), self.assertRaises(ValueError):
                probe.finite_vector(values, "test")

    def test_publisher_isolation_accepts_phase_specific_probe_node(self):
        checker = probe.MotionChecks.__new__(probe.MotionChecks)
        checker.node = NS(get_publishers_info_by_topic=lambda topic: [
            NS(node_name="linorobot2_g3_safety_probe"), NS(node_name="command_timeout")])
        checker.report = {}
        checker.assert_publishers_isolated("linorobot2_g3_safety_probe")
        self.assertEqual(checker.report["cmd_vel_publishers"],
                         ["command_timeout", "linorobot2_g3_safety_probe"])
        with self.assertRaisesRegex(RuntimeError, "unexpected"):
            checker.assert_publishers_isolated()


class OdomTests(unittest.TestCase):
    @staticmethod
    def odom(frame="odom", child="base_footprint"):
        vec = lambda x=0.0, y=0.0, z=0.0: NS(x=x, y=y, z=z)
        return NS(header=NS(frame_id=frame), child_frame_id=child,
                  pose=NS(pose=NS(position=vec(1, 2, 0),
                                  orientation=NS(x=0, y=0, z=0, w=1))),
                  twist=NS(twist=NS(linear=vec(0.1, 0, 0), angular=vec(0, 0, 0.2))))

    def test_odom_values_validate_frames_and_capture_motion_fields(self):
        value = probe.MotionChecks.odom_values(self.odom())
        self.assertEqual(value["position"], (1.0, 2.0, 0.0))
        self.assertAlmostEqual(value["linear"][0], 0.1)
        self.assertAlmostEqual(value["angular"][2], 0.2)
        checker = probe.MotionChecks.__new__(probe.MotionChecks)
        checker.odom_samples = {"filtered": []}
        checker.error = None
        checker.on_odom("filtered", self.odom())
        self.assertEqual(len(checker.odom_samples["filtered"]), 1)
        checker.on_odom("filtered", self.odom(frame="map"))
        self.assertIn("frame contract", checker.error)
        checker = probe.MotionChecks.__new__(probe.MotionChecks)
        checker.odom_samples = {"unfiltered": []}
        checker.error = None
        raw = self.odom(child="base_drive")
        checker.on_odom("unfiltered", raw)
        self.assertEqual(checker.odom_samples["unfiltered"][0]["child_frame_id"], "base_drive")

    def test_bad_odometry_quaternion_and_velocity_rejected(self):
        value = self.odom()
        value.pose.pose.orientation.w = 0
        with self.assertRaisesRegex(ValueError, "unit"):
            probe.MotionChecks.odom_values(value)
        value = self.odom()
        value.twist.twist.angular.z = math.inf
        with self.assertRaisesRegex(ValueError, "finite"):
            probe.MotionChecks.odom_values(value)

    def test_cli_rejects_unbounded_segments_and_bad_supervisor(self):
        for args in (("--segment-seconds", "0.1"), ("--segment-seconds", "6"),
                     ("--segment-seconds", "nan"), ("--supervisor-pid", "0")):
            with self.subTest(args=args), mock.patch("sys.stderr") as stderr, \
                    self.assertRaises(SystemExit) as error:
                probe.main(["--supervisor-pid", "10", "--report", "unused", *args])
            self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
