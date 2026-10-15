#!/usr/bin/env python3
"""ROS-free regression tests: python3 scripts/test_lidar_qos.py."""

import builtins
import contextlib
import copy
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock


spec = importlib.util.spec_from_file_location(
    "lidar_qos_under_test", Path(__file__).with_name("verify_lidar_qos.py"))
probe = importlib.util.module_from_spec(spec)
with mock.patch.object(sys, "dont_write_bytecode", True):
    spec.loader.exec_module(probe)


def quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def matrix_rotate(vector, roll, pitch, yaw):
    """Independent Rz Ry Rx fixture, not production quaternion rotation."""
    x, y, z = vector
    y, z = math.cos(roll)*y - math.sin(roll)*z, math.sin(roll)*y + math.cos(roll)*z
    x, z = math.cos(pitch)*x + math.sin(pitch)*z, -math.sin(pitch)*x + math.cos(pitch)*z
    return (math.cos(yaw)*x - math.sin(yaw)*y,
            math.sin(yaw)*x + math.cos(yaw)*y, z)


def independent_range(origin, direction):
    """Enumerate six faces + ground rather than reusing the slab algorithm."""
    hits = []
    low, high = (1.5, -0.5, 0.0), (2.5, 0.5, 1.0)
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            continue
        for face in (low[axis], high[axis]):
            t = (face - origin[axis]) / direction[axis]
            point = tuple(o + t*d for o, d in zip(origin, direction))
            if t >= 0 and all(low[j] - 1e-10 <= point[j] <= high[j] + 1e-10
                              for j in range(3)):
                hits.append(t)
    if direction[2] < -1e-12:
        hits.append(-origin[2]/direction[2])
    distance = min(hits, default=math.inf)
    return distance if distance <= 12.0 else math.inf


def fixture(position=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    offset = matrix_rotate((0.02, 0.0, 0.14875), *rpy)
    origin = tuple(a + b for a, b in zip(position, offset))
    sensor = probe.pose(origin, quaternion(*rpy))
    increment = (3.14159 - (-3.14159)) / 359
    values = []
    for i in range(360):
        a = -3.14159 + i*increment
        values.append(independent_range(origin, matrix_rotate((math.cos(a), math.sin(a), 0), *rpy)))
    scan = NS(header=NS(frame_id="laser", stamp=NS(sec=1, nanosec=0)),
              ranges=values, angle_min=-3.14159, angle_max=3.14159,
              angle_increment=increment, range_min=0.08, range_max=12.0)
    return scan, sensor


def pose_json(**changes):
    entry = {"name": probe.MODEL, "position": {}, "orientation": {"w": 1.0}}
    entry.update(changes)
    return json.dumps({"pose": [entry]})


class GeometryTests(unittest.TestCase):
    def test_normal_rays_and_non_wrapped_forward_indices(self):
        scan, sensor = fixture()
        result = probe.compare_scan(scan, sensor)
        self.assertTrue(result["pass"], result)
        self.assertIn(result["center_index"], (179, 180))
        self.assertEqual(result["box_rays"], 38)
        self.assertAlmostEqual(scan.ranges[179], 1.48/math.cos(scan.angle_min + 179*scan.angle_increment))
        self.assertEqual(scan.ranges[0], math.inf)
        self.assertEqual(scan.ranges[359], math.inf)
        self.assertLess(result["max_error_m"], 1e-10)

    def test_shifted_yawed_pitched_and_rolled_world_poses(self):
        for position, rpy in (((0.2, 0.1, 0.1), (0, 0, 0)),
                              ((0, 0, 0), (0, 0, 0.6)),
                              ((0, 0, 0.1), (0, 0.1, 0.2)),
                              ((0.1, -0.1, 0.1), (0.08, 0.1, -0.3))):
            with self.subTest(position=position, rpy=rpy):
                scan, sensor = fixture(position, rpy)
                result = probe.compare_scan(scan, sensor)
                self.assertTrue(result["pass"], result)
                self.assertLess(result["max_error_m"], 1e-9)
                if rpy[1]:
                    self.assertGreater(result["ground_rays"], 0)

    def test_composition_rotates_translation_and_all_orientation_axes(self):
        parent = probe.pose((0.2, -0.3, 0.4), quaternion(0.1, 0.2, 0.3))
        child = probe.pose((0.02, 0, 0.14875), quaternion(-0.2, 0.4, -0.1))
        result = probe.compose(parent, child)
        offset = matrix_rotate(child["position"], 0.1, 0.2, 0.3)
        for actual, expected in zip(result["position"], (a+b for a, b in zip(parent["position"], offset))):
            self.assertAlmostEqual(actual, expected)
        expected = matrix_rotate(matrix_rotate((0.3, 0.4, 0.5), -0.2, 0.4, -0.1), 0.1, 0.2, 0.3)
        for actual, value in zip(probe.rotate(result["orientation"], (0.3, 0.4, 0.5)), expected):
            self.assertAlmostEqual(actual, value)

    def test_ray_box_parallel_behind_inside_and_top(self):
        cases = (((0, 0, 0.2), (1, 0, 0), 1.5),
                 ((0, 1, 0.2), (1, 0, 0), math.inf),
                 ((3, 0, 0.2), (1, 0, 0), math.inf),
                 ((2, 0, 0.2), (1, 0, 0), 0.5),
                 ((2, 0, 2), (0, 0, -1), 1.0),
                 ((0, 0, 2), (1, 0, 0), math.inf))
        for origin, direction, expected in cases:
            with self.subTest(origin=origin, direction=direction):
                self.assertEqual(probe.ray_box(origin, direction), expected)

    def test_nearest_ground_not_box_for_pitched_down_ray(self):
        sensor = probe.pose((0.02, 0, 0.14875), quaternion(pitch=0.5))
        prediction = probe.predict_rays(sensor, 0, 0.01, count=1)[0]
        self.assertEqual(prediction[1], "ground")
        self.assertAlmostEqual(prediction[0], 0.14875/math.sin(0.5))

    def test_invalid_frame_shape_range_angles_and_increment(self):
        cases = (("range_min", 0.081), ("range_max", 11.99), ("angle_min", 0),
                 ("angle_max", math.pi - 0.01), ("angle_increment", 2*math.pi/360),
                 ("angle_increment", -0.01), ("angle_increment", math.nan),
                 ("range_min", math.inf))
        for key, value in cases:
            with self.subTest(key=key, value=value):
                scan, sensor = fixture()
                setattr(scan, key, value)
                with self.assertRaises(ValueError):
                    probe.compare_scan(scan, sensor)
        for count in (0, 359, 361):
            scan, sensor = fixture()
            scan.ranges = [math.inf]*count
            with self.assertRaisesRegex(ValueError, "exactly 360"):
                probe.compare_scan(scan, sensor)
        scan, sensor = fixture()
        scan.header.frame_id = "base_laser"
        with self.assertRaisesRegex(ValueError, "frame"):
            probe.compare_scan(scan, sensor)

    def test_small_float_metadata_noise_accepted(self):
        scan, sensor = fixture()
        scan.angle_min += 1e-6
        scan.angle_max += 1e-6
        scan.range_max += 1e-6
        self.assertTrue(probe.compare_scan(scan, sensor)["pass"])

    def test_wrong_forward_wrap_or_wrong_world_pose_fails(self):
        scan, sensor = fixture()
        scan.ranges = scan.ranges[180:] + scan.ranges[:180]
        self.assertFalse(probe.compare_scan(scan, sensor)["pass"])
        scan, sensor = fixture()
        wrong = probe.pose((0.3, 0, 0.14875), (0, 0, 0, 1))
        self.assertFalse(probe.compare_scan(scan, wrong)["pass"])

    def test_all_infinite_is_not_a_geometric_pass(self):
        scan, sensor = fixture()
        scan.ranges = [math.inf]*360
        result = probe.compare_scan(scan, sensor)
        self.assertFalse(result["pass"])
        self.assertEqual(result["matched_box_rays"], 0)
        self.assertGreater(len(result["mismatched_indices"]), 20)

    def test_no_box_hit_or_too_few_box_rays_fails(self):
        for position in ((-11, 0, 0), (0, 0, 2), (-6, 0, 0)):
            scan, sensor = fixture(position)
            result = probe.compare_scan(scan, sensor)
            self.assertFalse(result["pass"])
            self.assertLess(result["box_rays"], 20)

    def test_nan_negative_infinity_and_out_of_range_never_exempt(self):
        for index in (0, 161, 179):
            for value in (math.nan, -math.inf, -1, 0.01, 12.1):
                with self.subTest(index=index, value=value):
                    scan, sensor = fixture()
                    scan.ranges[index] = value
                    result = probe.compare_scan(scan, sensor)
                    self.assertFalse(result["pass"])
                    self.assertIn(index, result["invalid_indices"])

    def test_no_hit_positive_infinity_or_max_saturation_only(self):
        scan, sensor = fixture()
        scan.ranges[0] = 12.0
        self.assertTrue(probe.compare_scan(scan, sensor)["pass"])
        scan.ranges[0] = 4.0
        self.assertFalse(probe.compare_scan(scan, sensor)["pass"])
        scan, sensor = fixture()
        scan.ranges[179] = math.inf
        self.assertFalse(probe.compare_scan(scan, sensor)["pass"])

    def test_distance_tolerance_and_interior_mismatch_even_one_fails(self):
        scan, sensor = fixture()
        scan.ranges[179] += 0.049
        self.assertTrue(probe.compare_scan(scan, sensor)["pass"])
        scan.ranges[179] += 0.002
        result = probe.compare_scan(scan, sensor)
        self.assertFalse(result["pass"])
        self.assertEqual(result["mismatched_indices"], [179])

    def test_at_most_two_adjacent_silhouette_ray_mismatches(self):
        scan, sensor = fixture()
        hits = [i for i, x in enumerate(scan.ranges) if math.isfinite(x)]
        first, last = min(hits), max(hits)
        scan.ranges[first] = scan.ranges[last] = math.inf
        result = probe.compare_scan(scan, sensor)
        self.assertTrue(result["pass"], result)
        self.assertEqual(len(result["boundary_mismatched_indices"]), 2)
        scan.ranges[first - 1] = fixture()[0].ranges[first]
        self.assertFalse(probe.compare_scan(scan, sensor)["pass"])

    def test_arbitrary_error_at_boundary_not_tolerated(self):
        scan, sensor = fixture()
        first = next(i for i, x in enumerate(scan.ranges) if math.isfinite(x))
        scan.ranges[first] = 8.0
        result = probe.compare_scan(scan, sensor)
        self.assertFalse(result["pass"])
        self.assertIn(first, result["mismatched_indices"])

    def test_quaternion_validation_and_noise_normalization(self):
        for q in ((0, 0, 0, 0), (0, 0, 0, 2), (math.nan, 0, 0, 1), (0, 0, 1)):
            with self.subTest(q=q), self.assertRaises(ValueError):
                probe.unit_quaternion(q)
        self.assertEqual(probe.unit_quaternion((0, 0, 0, 1.0001)), (0, 0, 0, 1))
        with self.assertRaises(ValueError):
            probe.predict_rays(probe.pose((0, 0, 0), (0, 0, 0, 1)), -math.pi, 0.1)

    def test_pose_stability_translation_angle_and_quaternion_sign(self):
        start = probe.pose((0, 0, 0), (0, 0, 0, 1))
        self.assertEqual(probe.require_stable_pose(start, probe.pose((0, 0, 0), (0, 0, 0, -1))),
                         {"translation_m": 0, "quaternion_angle_rad": 0})
        probe.require_stable_pose(start, probe.pose((0.019, 0, 0), quaternion(yaw=0.029)))
        for end in (probe.pose((0.0201, 0, 0), (0, 0, 0, 1)),
                    probe.pose((0, 0, 0), quaternion(pitch=0.0301))):
            with self.assertRaisesRegex(ValueError, "moved"):
                probe.require_stable_pose(start, end)

    def test_stamp_sequence_and_invalid_stamp(self):
        scan, _ = fixture()
        self.assertEqual(probe.scan_stamp_ns(scan), 1_000_000_000)
        probe.require_increasing_stamp(None, 0)
        for i in range(1, 11):
            probe.require_increasing_stamp(i - 1, i)
        for current in (10, 9):
            with self.assertRaises(ValueError):
                probe.require_increasing_stamp(10, current)
        scan.header.stamp.nanosec = 1_000_000_000
        with self.assertRaises(ValueError):
            probe.scan_stamp_ns(scan)


class PoseParserTests(unittest.TestCase):
    def test_omitted_zero_fields_default_to_zero(self):
        self.assertEqual(probe.parse_model_pose(pose_json()),
                         probe.pose((0, 0, 0), (0, 0, 0, 1)))
        self.assertEqual(probe.parse_model_pose(pose_json(position={"z": 0.1}))["position"],
                         (0, 0, 0.1))
        # Missing w is valid only for a real unit quaternion (180-degree rotation).
        self.assertEqual(probe.parse_model_pose(pose_json(orientation={"x": 1}))["orientation"],
                         (1, 0, 0, 0))

    def test_exact_model_only_no_link_or_prefix_match(self):
        for name in ("linorobot2_2wd::base_link", "linorobot2_2wd_extra", "other"):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "exact model"):
                probe.parse_model_pose(pose_json(name=name))

    def test_malformed_missing_duplicated_and_invalid_quaternion(self):
        entry = json.loads(pose_json())["pose"][0]
        cases = ("not JSON", "{}", "null", '{"pose":{}}', '{"pose":[]}',
                 json.dumps({"pose": [entry, entry]}),
                 pose_json(orientation={}), pose_json(orientation={"w": 2}),
                 pose_json(orientation={"w": "invalid"}),
                 pose_json(position={"x": math.nan}), pose_json(position=None),
                 json.dumps({"pose": [{"name": probe.MODEL, "position": {}}]}))
        for text in cases:
            with self.subTest(text=text), self.assertRaises(ValueError):
                probe.parse_model_pose(text)

    def test_bounded_dynamic_command_and_fallback(self):
        outputs = [NS(returncode=0, stdout='{"pose": []}', stderr=""),
                   NS(returncode=0, stdout=pose_json(), stderr="")]
        with mock.patch.object(probe.time, "monotonic", return_value=0), \
                mock.patch.object(probe.subprocess, "run", side_effect=outputs) as run:
            evidence = probe.read_world_pose(20)
        self.assertTrue(evidence["topic"].endswith("/pose/info"))
        self.assertEqual(run.call_args_list[0].args[0],
                         ["ign", "topic", "-e", "-t", "/world/fortress_test/dynamic_pose/info",
                          "-n", "1", "--json-output"])
        for call in run.call_args_list:
            self.assertEqual(call.kwargs["timeout"], 5.0)
            self.assertTrue(call.kwargs["capture_output"])

    def test_timeout_diagnostic_never_spawn_fallback(self):
        with mock.patch.object(probe.time, "monotonic", return_value=0), \
                mock.patch.object(probe.subprocess, "run", side_effect=subprocess.TimeoutExpired("ign", 5)):
            with self.assertRaisesRegex(ValueError, "no measured Gazebo.*timeout"):
                probe.read_world_pose(20)

    def test_remaining_deadline_caps_subprocess_timeout(self):
        with mock.patch.object(probe.time, "monotonic", return_value=19), \
                mock.patch.object(probe.subprocess, "run", return_value=NS(
                    returncode=0, stdout=pose_json(), stderr="")) as run:
            probe.read_world_pose(20)
            self.assertEqual(run.call_args.kwargs["timeout"], 1)


def qos(reliability):
    return NS(reliability=NS(name=reliability), history=NS(name="KEEP_LAST"),
              depth=10, durability=NS(name="VOLATILE"))


def endpoint(name, reliability, gid=1):
    return NS(node_name=name, node_namespace="/", endpoint_gid=bytes([gid]*16),
              qos_profile=qos(reliability))


def mock_checker(pub, sub):
    # The mock verifies wiring and fail-closed behavior, not the real RMW API.
    incompatible = pub.reliability.name == "BEST_EFFORT" and sub.reliability.name == "RELIABLE"
    return NS(name="ERROR" if incompatible else "OK"), "reliability" if incompatible else ""


class QoSTests(unittest.TestCase):
    def setUp(self):
        self.pubs = [endpoint("parameter_bridge", "RELIABLE")]
        self.subs = [endpoint("probe", "BEST_EFFORT", 2), endpoint("probe", "RELIABLE", 3)]
        self.rviz = endpoint("rviz2", "BEST_EFFORT", 4)
        self.node = NS(get_name=lambda: "probe", get_namespace=lambda: "/",
                       get_publishers_info_by_topic=lambda topic: self.pubs,
                       get_subscriptions_info_by_topic=lambda topic:
                           self.subs + ([self.rviz] if topic == "/scan" else []))
        self.counts = {topic: {"best_effort": 1, "reliable": 1} for topic in probe.TOPICS}

    def test_inventory_contains_actual_qos_gid_and_delivery(self):
        inventory, failures = probe.qos_inventory(self.node, mock_checker, self.counts)
        self.assertFalse(failures)
        self.assertEqual(len(inventory), 5)
        scan = inventory["/scan"]
        self.assertEqual(scan["publishers"][0]["qos"]["reliability"], "RELIABLE")
        self.assertEqual(scan["publishers"][0]["endpoint_gid"], "01"*16)
        self.assertEqual(len(scan["selected_compatibility"]), 3)
        self.assertEqual(scan["samples"], {"best_effort": 1, "reliable": 1})

    def test_best_effort_accepts_both_publisher_reliabilities(self):
        sub = endpoint("probe", "BEST_EFFORT")
        for reliability in ("RELIABLE", "BEST_EFFORT"):
            evidence = probe.compatibility_evidence(endpoint("bridge", reliability), sub, mock_checker)
            self.assertTrue(evidence["pass"])

    def test_reliable_incompatibility_is_a_contract_failure(self):
        self.pubs[0] = endpoint("bridge", "BEST_EFFORT")
        inventory, errors = probe.qos_inventory(self.node, mock_checker, self.counts)
        self.assertTrue(any("contract fix required" in x for x in errors))
        self.assertTrue(inventory["/scan"]["selected_compatibility"][0]["pass"])
        self.assertFalse(inventory["/scan"]["selected_compatibility"][1]["pass"])

    def test_warning_unknown_and_error_compatibility_rejected(self):
        for status in ("WARNING", "UNKNOWN", "ERROR"):
            with self.subTest(status=status):
                checker = lambda *args: (NS(name=status), "uncertain endpoint")
                _, errors = probe.qos_inventory(self.node, checker, self.counts)
                self.assertTrue(any("contract fix required" in x for x in errors))

    def test_no_publishers_missing_rviz_subscription_and_missing_samples_fail(self):
        self.pubs.clear()
        _, errors = probe.qos_inventory(self.node, mock_checker, self.counts)
        self.assertEqual(sum("no publishers" in x for x in errors), 5)
        self.pubs.append(endpoint("bridge", "RELIABLE"))
        self.node.get_subscriptions_info_by_topic = lambda topic: self.subs
        _, errors = probe.qos_inventory(self.node, mock_checker, self.counts)
        self.assertIn("/scan: required rviz2 subscription absent", errors)
        self.counts["/camera/image_raw"]["reliable"] = 0
        _, errors = probe.qos_inventory(self.node, mock_checker, self.counts)
        self.assertIn("/camera/image_raw: no reliable subscription sample", errors)

    def test_actual_incompatible_rviz_profile_fails(self):
        self.pubs[0] = endpoint("bridge", "BEST_EFFORT")
        self.rviz.qos_profile = qos("RELIABLE")
        _, errors = probe.qos_inventory(self.node, mock_checker, self.counts)
        self.assertTrue(any("rviz2 QoS ERROR" in x for x in errors))


class MainTests(unittest.TestCase):
    def test_import_does_not_require_ros(self):
        # The module has just loaded on macOS without importing ROS packages.
        self.assertNotIn("rclpy", probe.__dict__)
        self.assertNotIn("tf2_ros", probe.__dict__)

    def test_wrong_or_missing_domain_blocks_runtime_and_reports_failure(self):
        for domain in (None, "44", "042"):
            env = {} if domain is None else {"ROS_DOMAIN_ID": domain}
            with self.subTest(domain=domain), mock.patch.dict(os.environ, env, clear=True), \
                    mock.patch.object(probe, "run_probe") as run, tempfile.TemporaryDirectory() as tmp, \
                    contextlib.redirect_stdout(io.StringIO()):
                path = Path(tmp)/"report.json"
                self.assertEqual(probe.main(["--report", str(path)]), 1)
                run.assert_not_called()
                result = json.loads(path.read_text())
                self.assertFalse(result["pass"])
                self.assertIn("before rclpy.init", result["errors"][0])

    def test_partial_exception_returns_nonzero_with_json_evidence(self):
        with mock.patch.dict(os.environ, {"ROS_DOMAIN_ID": "42"}), \
                mock.patch.object(probe, "run_probe", side_effect=ValueError("no TF")), \
                tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            path = Path(tmp)/"failure.json"
            self.assertEqual(probe.main(["--report", str(path)]), 1)
            self.assertIn("no TF", json.loads(path.read_text())["errors"][0])

    def test_invalid_or_unbounded_timeout_rejected(self):
        for timeout in ("nan", "inf", "-1", "19", "61"):
            with self.subTest(timeout=timeout), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    probe.main(["--timeout", timeout])
                self.assertEqual(error.exception.code, 2)


class RuntimeTests(unittest.TestCase):
    """Real orchestration, stubbed runtime imports; no real ROS/processes/waits."""

    def setUp(self):
        self.now = 0.0
        self.active = False
        self.scan_limit = 10
        self.spins = 0
        self.no_tf = False
        self.repeat_stamp = False
        self.skip_reliable_topic = None
        self.end_position = (0, 0, 0)
        self.report = {}
        self.created = None
        self.lookups = []
        self.sample, _ = fixture()
        owner = self

        class FakeNode:
            def __init__(self, name, **kwargs):
                owner.created = self
                self.name = name
                self.subscriptions = []
                self.destroy_node = mock.Mock()

            def get_name(self):
                return self.name

            def get_namespace(self):
                return "/"

            def create_subscription(self, msg_type, topic, callback, profile):
                self.subscriptions.append((topic, callback, profile))

            def get_publishers_info_by_topic(self, topic):
                return [endpoint("bridge", "RELIABLE")]

            def get_subscriptions_info_by_topic(self, topic):
                results = []
                for index, (name, _, profile) in enumerate(self.subscriptions):
                    if name == topic:
                        item = endpoint(self.name, profile.reliability.name, index + 1)
                        item.qos_profile = profile
                        results.append(item)
                if topic == "/scan":
                    results.append(endpoint("rviz2", "BEST_EFFORT"))
                return results

        class FakeTFError(Exception):
            pass

        class FakeBuffer:
            def lookup_transform(self, target, source, stamp, timeout):
                owner.lookups.append((target, source, stamp, timeout.seconds))
                if owner.no_tf:
                    raise FakeTFError("missing timestamped sensor TF")
                return NS(transform=NS(translation=NS(x=0.02, y=0.0, z=0.14875),
                                       rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))

        def init(**kwargs):
            self.active = True

        def spin(node, timeout_sec):
            self.now += 1
            self.spins += 1
            for topic, receive, profile in node.subscriptions:
                if topic == self.skip_reliable_topic and profile.reliability.name == "RELIABLE":
                    continue
                if topic == "/scan":
                    if self.spins > self.scan_limit:
                        continue
                    msg = copy.deepcopy(self.sample)
                    msg.header.stamp.sec = 1 if self.repeat_stamp else self.spins
                else:
                    msg = NS()
                receive(msg)

        self.ros = NS(init=mock.Mock(side_effect=init), ok=lambda: self.active,
                      spin_once=spin, shutdown=mock.Mock(side_effect=lambda: setattr(self, "active", False)))
        self.stubs = {
            "rclpy": self.ros, "rclpy.node": NS(Node=FakeNode),
            "rclpy.duration": NS(Duration=lambda **kwargs: NS(**kwargs)),
            "rclpy.time": NS(Time=NS(from_msg=lambda msg: msg)),
            "rclpy.qos": NS(QoSDurabilityPolicy=NS(VOLATILE=NS(name="VOLATILE")),
                            QoSHistoryPolicy=NS(KEEP_LAST=NS(name="KEEP_LAST")),
                            QoSReliabilityPolicy=NS(RELIABLE=NS(name="RELIABLE")),
                            QoSProfile=lambda **kwargs: NS(**kwargs),
                            qos_check_compatible=mock_checker,
                            qos_profile_sensor_data=qos("BEST_EFFORT")),
            "rosgraph_msgs.msg": NS(Clock=type("Clock", (), {})),
            "sensor_msgs.msg": NS(**{n: type(n, (), {}) for n in
                                     ("CameraInfo", "Image", "Imu", "LaserScan")}),
            "tf2_ros": NS(Buffer=FakeBuffer, TransformException=FakeTFError,
                          TransformListener=lambda *args, **kwargs: NS())}
        real_import = builtins.__import__

        def import_stub(name, globals=None, locals=None, fromlist=(), level=0):
            if level == 0 and name in self.stubs:
                return self.stubs[name]
            return real_import(name, globals, locals, fromlist, level)

        def world_pose(deadline):
            position = (0, 0, 0) if not self.report.get("world_pose_start") else self.end_position
            return {"topic": "/world/fortress_test/dynamic_pose/info",
                    "pose": probe.pose(position, (0, 0, 0, 1))}

        for patch in (mock.patch.object(builtins, "__import__", side_effect=import_stub),
                      mock.patch.object(probe, "time", NS(monotonic=lambda: self.now)),
                      mock.patch.object(probe, "read_world_pose", side_effect=world_pose)):
            patch.start()
            self.addCleanup(patch.stop)

    def check_cleanup(self):
        self.created.destroy_node.assert_called_once_with()
        self.ros.shutdown.assert_called_once_with()

    def test_ten_scans_with_timestamped_tf_and_all_delivery_profiles_pass(self):
        probe.run_probe(self.report, 60)
        self.check_cleanup()
        self.assertEqual(self.report["scan_stats"]["compared_scans"], 10)
        self.assertEqual(self.report["scan_stats"]["first_stamp_ns"], 1_000_000_000)
        self.assertEqual(self.report["scan_stats"]["last_stamp_ns"], 10_000_000_000)
        self.assertEqual(len(self.created.subscriptions), 10)
        self.assertEqual(len(self.lookups), 10)
        for target, source, stamp, timeout in self.lookups:
            self.assertEqual((target, source, timeout), ("base_footprint", "laser", 0.0))
            self.assertGreater(stamp.sec, 0)
        json.dumps(self.report, allow_nan=False)

    def test_one_sample_never_passes(self):
        self.scan_limit = 1
        with self.assertRaisesRegex(ValueError, "only 1/10 compared scans"):
            probe.run_probe(self.report, 60)
        self.assertLessEqual(self.now, 60)
        self.check_cleanup()

    def test_no_scan_never_passes(self):
        self.scan_limit = 0
        with self.assertRaisesRegex(ValueError, "only 0/10 compared scans"):
            probe.run_probe(self.report, 60)
        self.check_cleanup()

    def test_no_tf_never_passes_or_substitutes_static_offsets(self):
        self.no_tf = True
        with self.assertRaisesRegex(ValueError, "missing timestamped sensor TF"):
            probe.run_probe(self.report, 60)
        self.assertEqual(self.report["scans"], [])
        self.check_cleanup()

    def test_no_measured_pose_is_not_replaced_with_spawn_pose(self):
        with mock.patch.object(probe, "read_world_pose", side_effect=ValueError("pose timeout")):
            with self.assertRaisesRegex(ValueError, "pose timeout"):
                probe.run_probe(self.report, 60)
        self.assertFalse(self.lookups)
        self.check_cleanup()

    def test_repeated_stamps_fail_and_cleanup(self):
        self.repeat_stamp = True
        with self.assertRaisesRegex(ValueError, "not strictly increasing"):
            probe.run_probe(self.report, 60)
        self.check_cleanup()

    def test_pose_motion_after_ten_scans_fails(self):
        self.end_position = (0.03, 0, 0)
        with self.assertRaisesRegex(ValueError, "pose moved"):
            probe.run_probe(self.report, 60)
        self.assertEqual(len(self.report["scans"]), 10)
        self.check_cleanup()

    def test_missing_reliable_sample_even_with_geometry_pass_fails(self):
        self.skip_reliable_topic = "/camera/camera_info"
        with self.assertRaisesRegex(ValueError, "no reliable subscription sample"):
            probe.run_probe(self.report, 60)
        self.assertEqual(len(self.report["scans"]), 10)
        self.check_cleanup()

    def test_failed_init_does_not_shutdown_invalid_context(self):
        self.ros.init.side_effect = RuntimeError("init failed")
        with self.assertRaisesRegex(RuntimeError, "init failed"):
            probe.run_probe(self.report, 60)
        self.assertIsNone(self.created)
        self.ros.shutdown.assert_not_called()

    def test_cli_success_and_partial_failure_save_strict_json(self):
        for no_tf in (False, True):
            with self.subTest(no_tf=no_tf), tempfile.TemporaryDirectory() as tmp, \
                    mock.patch.dict(os.environ, {"ROS_DOMAIN_ID": "42"}), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.now = self.spins = 0
                self.no_tf = no_tf
                path = Path(tmp)/"evidence.json"
                self.assertEqual(probe.main(["--report", str(path)]), int(no_tf))
                evidence = json.loads(path.read_text())
                self.assertEqual(evidence["pass"], not no_tf)
                self.assertEqual(evidence["scan_stats"]["compared_scans"], 0 if no_tf else 10)


if __name__ == "__main__":
    unittest.main()