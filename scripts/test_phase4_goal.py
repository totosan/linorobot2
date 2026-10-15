"""ROS-free contract tests for the bounded G4 Nav2 goal probe."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location(
    "phase4_goal_under_test", Path(__file__).with_name("verify_phase4_goal.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
try:
    import yaml
except ImportError:
    yaml = None
try:
    config_spec = importlib.util.spec_from_file_location(
        "navigation_launch_under_test", Path(__file__).parents[1]
        / "linorobot2_navigation/launch/navigation.launch.py")
    navigation_launch = importlib.util.module_from_spec(config_spec)
    config_spec.loader.exec_module(navigation_launch)
except ImportError:
    navigation_launch = None


def grid(width=20, height=20, resolution=0.1, origin=(-1.0, -1.0), values=None,
        yaw=0.0, frame="map", roll=0.0):
    qx, qz = math.sin(roll/2), math.sin(yaw/2)
    qw = math.cos(roll/2)*math.cos(yaw/2)
    data = values if values is not None else [0]*(width*height)
    return NS(header=NS(frame_id=frame), info=NS(width=width, height=height,
              resolution=resolution, origin=NS(position=NS(x=origin[0], y=origin[1]),
                      orientation=NS(x=qx, y=0.0, z=qz, w=qw))), data=data)


class GoalGeometryTests(unittest.TestCase):
    def test_incremental_pose_json_decoder_handles_pretty_split_and_concatenated_messages(self):
        document = {"pose": [{"name": "linorobot2_2wd",
                              "position": {"x": 1, "y": 2, "z": 0.1},
                              "orientation": {"w": 1}}]}
        pretty = json.dumps(document, indent=2)
        concatenated = json.dumps(document)+json.dumps(document)
        decoded = list(probe.iter_json_documents([pretty[:20], pretty[20:], "\n",
                                                  concatenated[:17], concatenated[17:]]))
        self.assertEqual(decoded, [document, document, document])
        with self.assertRaisesRegex(ValueError, "truncated"):
            list(probe.iter_json_documents(['{"pose": [']))

    def test_world_to_grid_supports_rotated_map_origin(self):
        metadata, data = probe.map_values(grid(yaw=math.pi/2))
        self.assertEqual(probe.occupancy_at(metadata, data, -1.5, -0.5), 0)
        self.assertIsNone(probe.occupancy_at(metadata, data, 10, 10))

    def test_goal_line_must_be_in_known_free_cells(self):
        metadata, data = probe.map_values(grid())
        samples = probe.validate_free_path(metadata, data, start=(0, 0), goal=(0.5, 0))
        self.assertGreater(len(samples), 2)
        self.assertTrue(all(sample["occupancy"] == 0 for sample in samples))
        with self.assertRaisesRegex(ValueError, "known free"):
            probe.validate_free_path(metadata, [-1]*(20*20), start=(0, 0), goal=(0.5, 0))
        occupied = [0]*(20*20)
        occupied[10*20+12] = 100
        with self.assertRaisesRegex(ValueError, "known free"):
            probe.validate_free_path(metadata, occupied, start=(0, 0), goal=(0.5, 0))

    def test_saved_map_validation_rejects_malformed_values_and_frames(self):
        for message in (grid(width=0), grid(resolution=0), grid(frame="odom"),
                        grid(values=[101]+[0]*(20*20-1)),
                        grid(origin=(math.nan, 0)), grid(roll=0.1)):
            with self.subTest(message=message), self.assertRaises(ValueError):
                probe.map_values(message)

    def test_velocity_commands_are_finite_and_within_g4_profile(self):
        msg = NS(linear=NS(x=0.08, y=0.0), angular=NS(z=0.30))
        self.assertEqual(probe.checked_command(msg), (0.08, 0.0, 0.30))
        for linear, lateral, angular in ((0.086, 0, 0), (0, 0.01, 0),
                                         (0, 0, 0.31), (math.nan, 0, 0)):
            bad = NS(linear=NS(x=linear, y=lateral), angular=NS(z=angular))
            with self.subTest(command=(linear, lateral, angular)), self.assertRaises(ValueError):
                probe.checked_command(bad)

    def test_localization_tf_must_be_current_and_temporally_plausible(self):
        clock_ns = 10_000_000_000
        stamps = {"map->odom": 9_500_000_000,
                  "odom->base_footprint": 9_900_000_000}
        ages = probe.validate_tf_freshness(clock_ns, stamps)
        self.assertEqual(ages, {"map->odom": 0.5, "odom->base_footprint": 0.1})
        tolerated_future = dict(stamps, **{"map->odom": 13_000_000_000})
        self.assertEqual(probe.validate_tf_freshness(clock_ns, tolerated_future)["map->odom"], -3.0)

        stale = dict(stamps, **{"map->odom": 8_900_000_000})
        with self.assertRaisesRegex(RuntimeError, "stale localization TF map->odom"):
            probe.validate_tf_freshness(clock_ns, stale)
        # Run 20 logged clock=5.143s and map->odom TF=2.619s (age=2.524s).
        run20_clock_ns = 5_143_000_000
        run20_stamps = {"map->odom": 2_619_000_000,
                        "odom->base_footprint": 5_100_000_000}
        with self.assertRaisesRegex(RuntimeError, "age=2.524s"):
            probe.validate_tf_freshness(run20_clock_ns, run20_stamps)
        with self.assertRaisesRegex(RuntimeError, "missing=.*map->odom"):
            probe.validate_tf_freshness(clock_ns, {"odom->base_footprint": clock_ns})
        too_far_future = dict(stamps, **{"map->odom": 14_000_000_001})
        with self.assertRaisesRegex(RuntimeError, "stale localization TF map->odom"):
            probe.validate_tf_freshness(clock_ns, too_far_future)

    def test_localization_snapshot_separates_sim_age_from_receive_age(self):
        snapshot = probe.localization_tf_snapshot(
            76_767_000_000,
            {"map->odom": 31_600_000_000,
             "odom->base_footprint": 76_739_000_000},
            {"map->odom": 100.0, "odom->base_footprint": 104.5},
            now=105.0)
        self.assertEqual(snapshot["transforms"]["map->odom"], {
            "stamp_ns": 31_600_000_000,
            "age_seconds": 45.167,
            "receive_age_seconds": 5.0,
            "updates_during_goal": None,
            "stamp_delta_ns_during_goal": None,
        })
        self.assertEqual(snapshot["transforms"]["odom->base_footprint"], {
            "stamp_ns": 76_739_000_000,
            "age_seconds": 0.028,
            "receive_age_seconds": 0.5,
            "updates_during_goal": None,
            "stamp_delta_ns_during_goal": None,
        })
        missing = probe.localization_tf_snapshot(1_000_000_000, {}, now=10.0)
        self.assertIsNone(missing["transforms"]["map->odom"]["stamp_ns"])
        self.assertIsNone(missing["transforms"]["map->odom"]["age_seconds"])

    def test_localization_snapshot_reports_per_goal_tf_delivery_and_stamp_progress(self):
        snapshot = probe.localization_tf_snapshot(
            20_000_000_000,
            {"map->odom": 10_000_000_000,
             "odom->base_footprint": 19_900_000_000},
            {"map->odom": 99.0, "odom->base_footprint": 104.5},
            now=105.0,
            tf_update_counts={"map->odom": 12, "odom->base_footprint": 30},
            tf_updates_at_start={"map->odom": 12, "odom->base_footprint": 25},
            tf_stamps_at_start={"map->odom": 10_000_000_000,
                                "odom->base_footprint": 18_000_000_000})
        self.assertEqual(snapshot["transforms"]["map->odom"]["updates_during_goal"], 0)
        self.assertEqual(snapshot["transforms"]["map->odom"]["stamp_delta_ns_during_goal"], 0)
        self.assertEqual(snapshot["transforms"]["odom->base_footprint"]["updates_during_goal"], 5)
        self.assertEqual(
            snapshot["transforms"]["odom->base_footprint"]["stamp_delta_ns_during_goal"],
            1_900_000_000)

    def test_amcl_pose_snapshot_reports_staleness_and_goal_progress(self):
        snapshot = probe.amcl_pose_snapshot(
            76_767_000_000, 31_600_000_000, 100.0, 9,
            count_at_start=9, stamp_at_start=31_600_000_000, now=105.0)
        self.assertEqual(snapshot, {
            "stamp_ns": 31_600_000_000,
            "age_seconds": 45.167,
            "receive_age_seconds": 5.0,
            "samples_during_goal": 0,
            "stamp_delta_ns_during_goal": 0,
        })
        missing = probe.amcl_pose_snapshot(1_000_000_000, None, None, 0, now=10.0)
        self.assertIsNone(missing["age_seconds"])
        self.assertIsNone(missing["receive_age_seconds"])

    def test_ground_truth_arrival_uses_footprint_tolerance_and_requires_final_zero(self):
        probe.validate_goal_arrival(0.199, 1)
        with self.assertRaisesRegex(RuntimeError, "limit=0.20m"):
            probe.validate_goal_arrival(0.212, 1)
        with self.assertRaisesRegex(RuntimeError, "limit=0.20m"):
            probe.validate_goal_arrival(0.201, 1)
        with self.assertRaisesRegex(RuntimeError, "zero_count=0"):
            probe.validate_goal_arrival(0.1, 0)
        with self.assertRaisesRegex(ValueError, "finite and nonnegative"):
            probe.validate_goal_arrival(math.inf, 1)

    def test_successful_goal_stop_accepts_final_zero_before_result_with_measured_stillness(self):
        commands = [
            {"time": 10.0, "values": (0.08, 0.0, 0.0), "zero": False},
            {"time": 11.8, "values": (0.0, 0.0, 0.0), "zero": True},
        ]
        samples = [
            {"time": 12.1, "pose": {"position": (0.44, 0.0, 0.0)}},
            {"time": 12.3, "pose": {"position": (0.441, 0.0, 0.0)}},
            {"time": 12.5, "pose": {"position": (0.442, 0.0, 0.0)}},
        ]
        evidence = probe.validate_successful_goal_stop(commands, samples, 12.0)
        self.assertEqual(evidence["stop_basis"],
                         "zero_after_last_nonzero_plus_post_result_stillness")
        self.assertEqual(evidence["zero_command_count_after_result"], 0)
        self.assertEqual(evidence["zero_command_count_after_last_nonzero"], 1)
        self.assertAlmostEqual(evidence["post_result_max_drift_m"], 0.002)
        probe.validate_goal_arrival(
            0.106, evidence["zero_command_count_after_last_nonzero"])

    def test_successful_goal_stop_rejects_missing_or_misordered_zero(self):
        samples = [
            {"time": time, "pose": {"position": (0.44, 0.0, 0.0)}}
            for time in (12.1, 12.3, 12.5)
        ]
        events = (
            [],
            [{"time": 9.0, "values": (0.0, 0.0, 0.0), "zero": True},
             {"time": 10.0, "values": (0.08, 0.0, 0.0), "zero": False}],
            [{"time": 10.0, "values": (0.08, 0.0, 0.0), "zero": False},
             {"time": 12.1, "values": (0.08, 0.0, 0.0), "zero": False}],
        )
        for commands in events:
            with self.subTest(commands=commands), self.assertRaises(RuntimeError):
                probe.validate_successful_goal_stop(commands, samples, 12.0)

    def test_successful_goal_stop_accepts_post_result_nonzero_only_if_followed_by_final_zero(self):
        commands = [
            {"time": 10.0, "values": (0.08, 0.0, 0.0), "zero": False},
            {"time": 11.8, "values": (0.0, 0.0, 0.0), "zero": True},
            {"time": 12.1, "values": (0.01, 0.0, 0.0), "zero": False},
            {"time": 12.2, "values": (0.0, 0.0, 0.0), "zero": True},
        ]
        samples = [
            {"time": time, "pose": {"position": (0.44, 0.0, 0.0)}}
            for time in (12.3, 12.5, 12.7)
        ]
        evidence = probe.validate_successful_goal_stop(commands, samples, 12.0)
        self.assertEqual(evidence["nonzero_command_count_after_result"], 1)
        self.assertEqual(evidence["zero_command_count_after_result"], 1)
        self.assertEqual(evidence["final_zero_command"], commands[-1])

    def test_successful_goal_stop_requires_multiple_still_samples_within_tolerance(self):
        command = [
            {"time": 10.0, "values": (0.08, 0.0, 0.0), "zero": False},
            {"time": 11.0, "values": (0.0, 0.0, 0.0), "zero": True},
        ]
        with self.assertRaisesRegex(RuntimeError, "three post-result"):
            probe.validate_successful_goal_stop(command, [], 12.0)
        moved = [
            {"time": time, "pose": {"position": (x, 0.0, 0.0)}}
            for time, x in ((12.1, 0.44), (12.3, 0.45), (12.5, 0.47))
        ]
        with self.assertRaisesRegex(RuntimeError, "did not remain stationary"):
            probe.validate_successful_goal_stop(command, moved, 12.0)

    def test_nav2_footprints_detect_frozen_internal_pose_despite_fresh_tf(self):
        # Run 23: live /tf advanced, but Nav2 later requested pose time 47.411s.
        clock = 60_726_000_000
        probe.validate_tf_freshness(clock, {
            "map->odom": clock+3_000_000_000, "odom->base_footprint": clock})
        footprints = {
            "local_costmap": {"frame_id": "odom", "stamp_ns": 47_411_000_000,
                              "received": 100.0},
            "global_costmap": {"frame_id": "map", "stamp_ns": clock,
                               "received": 100.0}}
        snapshot = probe.nav2_footprint_snapshot(clock, footprints, now=100.1)
        self.assertAlmostEqual(snapshot["local_costmap"]["age_seconds"], 13.315)
        with self.assertRaisesRegex(RuntimeError, "stale Nav2 footprint local_costmap"):
            probe.validate_nav2_footprints(clock, footprints, now=100.1)

    def test_nav2_footprints_require_both_fresh_pose_and_delivery(self):
        clock = 10_000_000_000
        footprints = {name: {"frame_id": frame, "stamp_ns": clock,
                             "received": 100.0}
                      for name, frame in probe.NAV2_FOOTPRINT_FRAMES.items()}
        probe.validate_nav2_footprints(clock, footprints, now=100.1)
        with self.assertRaisesRegex(RuntimeError, "freshness unavailable"):
            probe.validate_nav2_footprints(clock, {}, now=100.1)
        with self.assertRaisesRegex(RuntimeError, "freshness unavailable"):
            probe.validate_nav2_footprints(None, footprints, now=100.1)
        with self.assertRaisesRegex(RuntimeError, "stale Nav2 footprint"):
            probe.validate_nav2_footprints(clock, footprints, now=102.01)
        for name in footprints:
            for stamp in (clock-1_000_000_001, clock+3_500_000_001):
                bad = {key: dict(value) for key, value in footprints.items()}
                bad[name]["stamp_ns"] = stamp
                with self.subTest(name=name, stamp=stamp), self.assertRaises(RuntimeError):
                    probe.validate_nav2_footprints(clock, bad, now=100.1)

    def test_run24_rejects_local_pose_lag_while_global_pose_and_tf_are_fresh(self):
        clock = 41_003_000_000
        probe.validate_tf_freshness(clock, {
            "map->odom": 43_900_000_000, "odom->base_footprint": 41_000_000_000})
        footprints = {
            "local_costmap": {"frame_id": "odom", "stamp_ns": 40_002_000_000,
                              "received": 199.856},
            "global_costmap": {"frame_id": "map", "stamp_ns": 40_922_000_000,
                               "received": 199.921}}
        snapshot = probe.nav2_footprint_snapshot(clock, footprints, now=200.0)
        self.assertAlmostEqual(snapshot["global_costmap"]["age_seconds"], 0.081)
        self.assertAlmostEqual(snapshot["local_costmap"]["receive_age_seconds"], 0.144)
        with self.assertRaisesRegex(RuntimeError, "local_costmap: pose_age=1.001s"):
            probe.validate_nav2_footprints(clock, footprints, now=200.0)

    def test_nav2_footprint_sample_validates_frame_polygon_and_timestamp(self):
        def message(frame="odom", sec=10, nanosec=0, points=None):
            return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
                      polygon=NS(points=points if points is not None else
                                 [NS(x=x, y=y, z=0.0) for x, y in
                                  ((0.0, 0.0), (0.2, 0.0), (0.0, 0.2))]))
        sample = probe.footprint_sample(message(), "odom", received=100.0)
        self.assertEqual(sample, {"frame_id": "odom", "stamp_ns": 10_000_000_000,
                                  "received": 100.0})
        for bad in (message(frame="map"), message(sec=-1),
                    message(nanosec=1_000_000_000), message(points=[]),
                    message(points=[NS(x=math.nan, y=0, z=0)]*3)):
            with self.subTest(message=bad), self.assertRaises(ValueError):
                probe.footprint_sample(bad, "odom")

    def test_goal_arrival_diagnostics_capture_target_and_measured_endpoint(self):
        final_zero = {"time": 123.0, "values": (0.0, 0.0, 0.0), "zero": True}
        diagnostics = probe.goal_arrival_diagnostics(
            "obstacle goal", (2.85, -0.90),
            {"position": (2.60, -0.82, 0.1)}, math.hypot(0.25, 0.08),
            [final_zero], 42,
            {"map->odom": [0.01, -0.02, 0.0]}, [2.61, -0.81, 0.0])
        self.assertEqual(diagnostics["nav2_action_status"], "SUCCEEDED")
        self.assertEqual(diagnostics["target_map_xy_m"], [2.85, -0.90])
        self.assertEqual(diagnostics["fortress_ground_truth_end_xy_m"], [2.60, -0.82])
        self.assertAlmostEqual(diagnostics["ground_truth_distance_to_goal_m"], math.hypot(0.25, 0.08))
        self.assertAlmostEqual(diagnostics["ground_truth_xy_error_vector_m"][0], -0.25)
        self.assertAlmostEqual(diagnostics["ground_truth_xy_error_vector_m"][1], 0.08)
        self.assertEqual(diagnostics["zero_command_count_after_result"], 1)
        self.assertEqual(diagnostics["final_zero_command"], final_zero)
        self.assertEqual(diagnostics["ground_truth_trajectory_samples"], 42)
        self.assertEqual(diagnostics["tf_transforms"], {"map->odom": [0.01, -0.02, 0.0]})
        self.assertEqual(diagnostics["amcl_pose"], [2.61, -0.81, 0.0])

    def test_fortress_nav2_goal_tolerances_are_tighter_than_ground_truth_gate(self):
        config = (Path(__file__).parents[1]/"linorobot2_navigation"
                  /"config/navigation_fortress.yaml").read_text()
        self.assertEqual(config.count("xy_goal_tolerance: 0.08"), 2)
        self.assertNotIn("xy_goal_tolerance: 0.12", config)
        self.assertIn("update_min_d: 0.02", config)
        self.assertIn("update_min_a: 0.02", config)

    @unittest.skipIf(yaml is None or navigation_launch is None,
                     "PyYAML and ROS 2 launch are available in the Humble runtime")
    def test_fortress_uses_humble_smac_planner_with_physical_footprint(self):
        root = Path(__file__).parents[1]/"linorobot2_navigation"
        common = root/"config/navigation_common.yaml"
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory)/"fortress.yaml"
            hardware_path = Path(directory)/"hardware.yaml"
            navigation_launch._compose_parameter_file(
                common, root/"config/navigation_fortress.yaml", config_path)
            navigation_launch._compose_parameter_file(
                common, root/"config/navigation.yaml", hardware_path)
            config = yaml.safe_load(config_path.read_text())
            hardware = yaml.safe_load(hardware_path.read_text())
        package = ET.parse(root/"package.xml").getroot()
        runtime_dependencies = {item.text for item in package.findall("exec_depend")}
        planner = config["planner_server"]["ros__parameters"]["GridBased"]
        self.assertEqual(planner["plugin"], "nav2_smac_planner/SmacPlanner2D")
        self.assertEqual(planner["cost_travel_multiplier"], 2.0)
        self.assertFalse(planner["allow_unknown"])
        self.assertFalse(planner["downsample_costmap"])
        self.assertIn("nav2_smac_planner", runtime_dependencies)
        for profile in (config, hardware):
            for name in ("global_costmap", "local_costmap"):
                params = profile[name][name]["ros__parameters"]
                self.assertEqual(params["robot_radius"], probe.ROBOT_RADIUS)
        self.assertNotEqual(config["planner_server"]["ros__parameters"]["GridBased"]["plugin"],
                            hardware["planner_server"]["ros__parameters"]
                            ["GridBased"]["plugin"])

    def test_passage_geometry_derives_from_radius_clearance_and_lateral_budget(self):
        config = (Path(__file__).parents[1]/"linorobot2_navigation"
                  /"config/navigation_fortress.yaml").read_text()
        resolution = 0.05
        half_width = probe.NARROW_PASSAGE_HALF_WIDTH
        self.assertAlmostEqual(
            half_width, probe.ROBOT_RADIUS+probe.MIN_ROBOT_CLEARANCE
            +probe.NARROW_PASSAGE_LATERAL_BUDGET)
        self.assertAlmostEqual(probe.NARROW_PASSAGE_WIDTH, 1.0)
        self.assertAlmostEqual(probe.NARROW_PASSAGE_LOW_Y, -1.60)
        self.assertAlmostEqual(probe.NARROW_PASSAGE_HIGH_Y, -0.60)
        self.assertGreaterEqual(probe.NARROW_PASSAGE_LATERAL_BUDGET, 2*resolution)
        self.assertGreaterEqual(probe.MIN_ROBOT_CLEARANCE, 0.15)
        inflation_radius = 0.45
        self.assertLess(inflation_radius, half_width)
        self.assertGreater(inflation_radius, probe.ROBOT_RADIUS+probe.MIN_ROBOT_CLEARANCE)
        self.assertEqual(config.count("inflation_radius: 0.45"), 2)
        self.assertNotIn("inflation_radius: 0.40", config)

    def test_pregoal_cmd_publishers_allow_only_known_nav2_and_watchdog_nodes(self):
        known = [NS(node_name="/behavior_server\n"),
                 NS(node_name="/command_timeout"),
                 NS(node_name="/velocity_smoother")]
        self.assertEqual(probe.validate_pregoal_cmd_publishers(known),
                         ["behavior_server", "command_timeout", "velocity_smoother"])
        with self.assertRaisesRegex(RuntimeError, "unexpected /cmd_vel publishers"):
            probe.validate_pregoal_cmd_publishers(known+[NS(node_name="/teleop")])
        with self.assertRaisesRegex(RuntimeError, "unexpected /cmd_vel publishers"):
            probe.validate_pregoal_cmd_publishers(known[:-1])

    def test_global_costmap_requires_single_global_costmap_publisher(self):
        self.assertEqual(probe.validate_global_costmap_publishers(
            ["/global_costmap/global_costmap"]), ["global_costmap/global_costmap"])
        for publishers in ([], ["/map_server"],
                           ["/global_costmap/global_costmap", "/other_node"]):
            with self.subTest(publishers=publishers), self.assertRaisesRegex(
                    RuntimeError, "global costmap publisher ownership"):
                probe.validate_global_costmap_publishers(publishers)

    def test_nav2_costmap_cell_classifier_distinguishes_inflation_from_lethal(self):
        cases = {None: "out_of_map", -1: "unknown", 0: "free", 49: "free",
                 50: "inflated", 99: "inflated", 100: "lethal"}
        for value, expected in cases.items():
            with self.subTest(value=value):
                self.assertEqual(probe.nav2_costmap_cell_kind(value), expected)
        for value in (-2, 101):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "must be"):
                probe.nav2_costmap_cell_kind(value)

    def test_unreachable_target_is_in_physical_box_and_unknown_or_occupied(self):
        metadata, data = probe.map_values(
            grid(width=60, height=40, origin=(-1.0, -2.0)))
        col, row = 30, 20  # Physical box center (2,0).
        for value in (-1, 65, 100):
            data[row*60+col] = value
            target = probe.validate_unreachable_target(metadata, data)
            self.assertEqual((target["x"], target["y"]), probe.OBSTACLE_CENTER)
            self.assertEqual(target["occupancy"], value)
        for value in (0, 49, 64):
            data[row*60+col] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                probe.validate_unreachable_target(metadata, data)
        small, values = probe.map_values(grid())
        with self.assertRaisesRegex(ValueError, "in-map"):
            probe.validate_unreachable_target(small, values)

    def test_unreachable_recovery_tracks_peak_not_only_final_displacement(self):
        def sample(x, y=0):
            return {"time": 1.0, "pose": {"position": [x, y, 0]}}
        start = {"position": [0, 0, 0]}
        motion = probe.validate_unreachable_motion(start, [sample(0), sample(-0.15)])
        self.assertAlmostEqual(motion["maximum_displacement_m"], 0.15)
        with self.assertRaisesRegex(RuntimeError, "unsafe unreachable recovery"):
            probe.validate_unreachable_motion(start, [sample(0.251), sample(0)])
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            probe.validate_unreachable_motion(start, [sample(math.nan)])

    def test_fail_closed_unreachable_policy_rejects_any_nonzero_command(self):
        self.assertEqual(probe.validate_no_motion_commands([
            {"zero": True, "values": (0.0, 0.0, 0.0)},
            {"zero": True, "values": (0.0, 0.0, 0.0)}]),
            {"command_samples": 2, "nonzero_command_samples": 0})
        with self.assertRaisesRegex(RuntimeError, "emitted nonzero /cmd_vel"):
            probe.validate_no_motion_commands([
                {"zero": True, "values": (0.0, 0.0, 0.0)},
                {"zero": False, "values": (0.01, 0.0, 0.0)}])

    def test_unreachable_result_requires_accepted_aborted_and_measured_stop(self):
        samples = [{"time": float(i), "pose": {"position": [0, 0, 0]}}
                   for i in range(3)]
        zeros = [{"zero": True, "values": (0, 0, 0)}]
        stopped = probe.validate_unreachable_result(True, 6, zeros, samples)
        self.assertEqual(stopped["maximum_displacement_m"], 0)
        no_command_stop = probe.validate_unreachable_result(True, 6, [], samples, [])
        self.assertFalse(no_command_stop["explicit_zero_after_result"])
        self.assertEqual(no_command_stop["stop_basis"], "no_nonzero_command_emitted")
        for accepted, status in ((False, 6), (True, 4), (True, 5), (True, 2)):
            with self.subTest(accepted=accepted, status=status), self.assertRaises(RuntimeError):
                probe.validate_unreachable_result(accepted, status, zeros, samples)
        for zero_commands, stopped_samples in (([], samples[:2]), (zeros, samples[:2])):
            with self.assertRaisesRegex(RuntimeError, "measured stopped samples"):
                probe.validate_unreachable_result(True, 6, zero_commands, stopped_samples)
        for command in ({"zero": False, "values": (0, 0, 0)},
                        {"zero": True, "values": (0.08, 0, 0)}, {"zero": True}):
            with self.subTest(command=command), self.assertRaises(RuntimeError):
                probe.validate_unreachable_result(True, 6, [command], samples)
        samples[-1]["pose"]["position"][0] = 0.021
        with self.assertRaisesRegex(RuntimeError, "unsafe unreachable recovery"):
            probe.validate_unreachable_result(True, 6, zeros, samples)
        with self.assertRaisesRegex(RuntimeError, "emitted nonzero /cmd_vel"):
            probe.validate_unreachable_result(True, 6, [], samples, [
                {"zero": False, "values": (0.01, 0.0, 0.0)}])
        source = Path(__file__).with_name("verify_phase4_goal.py").read_text()
        self.assertLess(source.index("unreachable abort lacks final zero within 3s"),
                        source.index("stop_observation_start = time.monotonic()"))

    def test_unreachable_harness_branch_skips_mapping_and_baseline(self):
        text = Path(__file__).with_name("test_phase4_gate.sh").read_text()
        branch = text.split('echo "[G4] Single unreachable-goal scenario', 1)[1].split(
            "printf '%s\\n' \"[G4] Fortress SLAM capture", 1)[0]
        self.assertIn("--scenario unreachable", branch)
        self.assertIn("--goal-timeout 90", branch)
        self.assertIn("--unreachable-behavior-tree", branch)
        self.assertIn('"$UNREACHABLE_BT"', branch)
        self.assertIn('start_nav2 "$UNREACHABLE_MAP"', branch)
        self.assertIn("exit 0", branch)
        self.assertNotIn("verify_phase4_mapping.py", branch)
        self.assertNotIn("--scenario baseline", branch)

    def test_fail_closed_behavior_tree_has_no_motion_recovery(self):
        root = Path(__file__).parents[1]/"linorobot2_navigation"
        tree_path = root/"behavior_trees/navigate_to_pose_fail_closed.xml"
        tree = ET.parse(tree_path).getroot()
        self.assertEqual(tree.attrib["main_tree_to_execute"], "MainTree")
        node_names = [element.tag for element in tree.iter()]
        self.assertIn("ComputePathToPose", node_names)
        self.assertIn("FollowPath", node_names)
        for forbidden in ("RecoveryNode", "BackUp", "Spin", "Wait", "ClearEntireCostmap"):
            self.assertNotIn(forbidden, node_names)
        cmake = (root/"CMakeLists.txt").read_text()
        self.assertIn("DIRECTORY launch rviz maps behavior_trees", cmake)
        self.assertIn("file(GLOB NAVIGATION_CONFIG_FILES CONFIGURE_DEPENDS", cmake)
        self.assertIn("configure_file(", cmake)
        verifier = Path(__file__).with_name("verify_phase4_goal.py").read_text()
        self.assertIn("goal.behavior_tree = str(args.unreachable_behavior_tree)", verifier)
        self.assertIn("validate_no_motion_commands(state[\"commands\"][command_sample_start:])",
                  verifier)

    def test_g4_run_reports_persist_in_workspace_by_default(self):
        text = Path(__file__).with_name("test_phase4_gate.sh").read_text()
        self.assertIn('LOG_PARENT="${G4_LOG_PARENT:-${WORKSPACE_DIR}/log/g4_runtime}"', text)
        self.assertIn('mkdir -p "$LOG_PARENT"', text)
        self.assertIn('mktemp -d "${LOG_PARENT}/g4-nav-', text)

    def test_unreachable_target_matches_static_physical_world_box(self):
        world = (Path(__file__).parents[1]/"linorobot2_gazebo"
                 /"worlds/fortress_test.sdf")
        box = ET.parse(world).find(".//model[@name='test_obstacle']")
        self.assertEqual(box.findtext("static"), "true")
        pose = [float(value) for value in box.findtext("pose").split()]
        size = [float(value) for value in box.findtext("link/collision/geometry/box/size").split()]
        self.assertEqual(tuple(pose[:2]), probe.OBSTACLE_CENTER)
        self.assertEqual(size[:2], [2*probe.OBSTACLE_HALF_SIZE]*2)

    def test_unreachable_abort_requires_target_specific_planner_evidence(self):
        message = "Planning algorithm GridBased failed to generate a valid path to (2.00, 0.00)"
        evidence = probe.validate_unreachable_planner_log(message+"\n"+message)
        self.assertEqual(evidence["failed_planner_attempts"], 2)
        for text in ("", message.replace("2.00", "2.80"),
                     "[ActionServer] Aborting handle."):
            with self.subTest(log=text), self.assertRaisesRegex(RuntimeError, "planner failure"):
                probe.validate_unreachable_planner_log(text)

    def test_unreachable_abort_does_not_mask_tf_or_costmap_faults(self):
        message = "Planning algorithm GridBased failed to generate a valid path to (2.00, 0.00)"
        for error in ("Transform data too old", "worldToMap failed", "Sensor origin",
                      "Controller patience", "Failed to obtain robot pose"):
            with self.subTest(error=error), self.assertRaises(RuntimeError):
                probe.validate_unreachable_planner_log(message+"\n"+error)

    def test_goal_timeout_summary_reports_motion_commands_and_planner_progress(self):
        def sample(x, y):
            return {"time": 1.0, "pose": {"position": [x, y, 0.0]}}

        plan = [NS(pose=NS(position=NS(x=x, y=y)))
                for x, y in ((0.5, 0.0), (1.0, 0.5), (2.0, 0.5))]
        summary = probe.summarize_goal_timeout(
            "obstacle goal", (2.0, 0.0), probe.time.monotonic()-4,
            {"position": (0.5, 0.0, 0.0)}, [sample(0.6, 0.0), sample(0.8, 0.1)],
            [{"zero": False, "values": (0.08, 0.0, 0.15)},
             {"zero": True, "values": (0.0, 0.0, 0.0)}],
            [{"poses": plan}], 120, 30)
        self.assertEqual(summary["ground_truth_samples"], 2)
        self.assertAlmostEqual(summary["ground_truth_displacement_m"], math.hypot(0.3, 0.1))
        self.assertEqual(summary["nonzero_cmd_vel_samples"], 1)
        self.assertAlmostEqual(summary["max_abs_linear_x_mps"], 0.08)
        self.assertAlmostEqual(summary["last_planner_path_length_m"], 1.0+math.sqrt(0.5))
        self.assertEqual(summary["sim_clock_advances"], 120)

    def test_obstacle_goal_selects_easternmost_known_free_safe_lane_cell(self):
        metadata, data = probe.map_values(
            grid(width=120, height=80, resolution=0.05, origin=(-1.0, -2.0)))
        row = math.floor((-1.05-metadata["origin"][1])/metadata["resolution"])
        col = math.floor((2.80-metadata["origin"][0])/metadata["resolution"])
        data[row*metadata["width"]+col] = -1
        goal = probe.select_known_free_avoidance_goal(metadata, data)
        self.assertEqual(goal, (2.80, -1.00, 0))

        for x, y in probe.AVOID_GOAL_CANDIDATES:
            col = math.floor((x-metadata["origin"][0])/metadata["resolution"])
            row = math.floor((y-metadata["origin"][1])/metadata["resolution"])
            data[row*metadata["width"]+col] = -1
        with self.assertRaisesRegex(ValueError, "no known-free, safe"):
            probe.select_known_free_avoidance_goal(metadata, data)

    def test_safe_grid_path_rejects_unknown_barriers_and_diagonal_corner_cutting(self):
        metadata, data = probe.map_values(
            grid(width=120, height=80, resolution=0.05, origin=(-1.0, -2.0)))
        self.assertTrue(probe.has_known_safe_grid_path(
            metadata, data, (0.55, 0.0), (2.8, -1.05)))
        wall_col = math.floor((1.5-metadata["origin"][0])/metadata["resolution"])
        for row in range(metadata["height"]):
            data[row*metadata["width"]+wall_col] = -1
        self.assertFalse(probe.has_known_safe_grid_path(
            metadata, data, (0.55, 0.0), (2.8, -1.05)))

        corner_metadata, corner_data = probe.map_values(
            grid(width=10, height=10, resolution=0.05, origin=(-1.0, 0.8),
                 values=[-1]*100))
        corner_data[5*10+5] = 0
        corner_data[6*10+6] = 0
        start = (-1.0+5.5*0.05, 0.8+5.5*0.05)
        goal = (-1.0+6.5*0.05, 0.8+6.5*0.05)
        self.assertFalse(probe.has_known_safe_grid_path(
            corner_metadata, corner_data, start, goal))

    def test_safe_grid_path_requires_robot_radius_inside_map_bounds(self):
        raw_metadata, raw_data = probe.map_values(
            grid(width=68, height=29, resolution=0.05, origin=(-0.28, -0.936)))
        self.assertFalse(probe.has_known_safe_grid_path(
            raw_metadata, raw_data, (0.55, 0.0), (2.80, -1.05)))
        prepared_metadata, prepared_data = probe.map_values(
            grid(width=74, height=39, resolution=0.05, origin=(-0.28, -1.436)))
        self.assertTrue(probe.has_known_safe_grid_path(
            prepared_metadata, prepared_data, (0.55, 0.0), (2.80, -1.05)))
        self.assertEqual(probe.select_known_free_avoidance_goal(
            prepared_metadata, prepared_data, start=(0.55, 0.0)), (2.80, -1.05, 0))

    def test_global_plan_reports_box_clearance_and_detour_length(self):
        metadata, data = probe.map_values(grid(width=60, height=40, origin=(-1.0, -2.0)))

        def pose(x, y):
            return NS(header=NS(frame_id="map"), pose=NS(position=NS(x=x, y=y)))

        detour = {"frame_id": "map", "poses": [pose(0.55, 0), pose(1.2, -0.9),
                     pose(2.9, -0.9), pose(3.0, -0.9)],
                  "data": data}
        result = probe.analyze_avoidance_path(metadata, detour, (0.55, 0), (3.0, -0.9))
        self.assertGreater(result["minimum_robot_clearance_m"], 0.15)
        self.assertGreater(result["length_m"], result["direct_distance_m"]*1.02)

        direct = {"frame_id": "map", "poses": [pose(0.55, 0), pose(3.0, -0.9)], "data": data}
        self.assertLess(probe.analyze_avoidance_path(
            metadata, direct, (0.55, 0), (3.0, -0.9))["minimum_robot_clearance_m"], 0.15)
        with self.assertRaisesRegex(ValueError, "map frame"):
            probe.analyze_avoidance_path(metadata,
                {"frame_id": "odom", "poses": detour["poses"], "data": data},
                (0.55, 0), (3.0, 0))

    def test_narrow_passage_requires_known_safe_route_and_gate_crossing(self):
        metadata, data = probe.map_values(
            grid(width=150, height=120, resolution=0.05, origin=(-1.0, -3.0)))
        for row in range(metadata["height"]):
            y = metadata["origin"][1]+(row+0.5)*metadata["resolution"]
            if probe.NARROW_PASSAGE_LOW_Y <= y <= probe.NARROW_PASSAGE_HIGH_Y:
                continue
            for col in range(metadata["width"]):
                x = metadata["origin"][0]+(col+0.5)*metadata["resolution"]
                if abs(x-probe.NARROW_GATE_X) <= probe.NARROW_GATE_HALF_THICKNESS:
                    data[row*metadata["width"]+col] = 100
        goal = probe.validate_narrow_passage_goal(metadata, data, (2.8, -1.05))
        self.assertEqual((goal["x"], goal["y"]), probe.NARROW_PASSAGE_GOAL)

        def pose(x, y):
            return NS(header=NS(frame_id="map"), pose=NS(position=NS(x=x, y=y)))

        path = {"frame_id": "map", "poses": [pose(2.8, -1.05), pose(3.5, -1.10),
                 pose(4.8, -1.10)], "data": data}
        analyzed = probe.analyze_narrow_passage_path(
            metadata, path, (2.8, -1.05), probe.NARROW_PASSAGE_GOAL)
        self.assertEqual(len(analyzed["passage_crossings"]), 1)
        self.assertAlmostEqual(
            analyzed["passage_crossings"][0]["minimum_wall_clearance_m"],
            probe.NARROW_PASSAGE_HALF_WIDTH-probe.ROBOT_RADIUS)
        self.assertAlmostEqual(analyzed["passage_width_m"], probe.NARROW_PASSAGE_WIDTH)

        unsafe_path = {"frame_id": "map", "poses": [pose(2.8, -0.70), pose(4.8, -0.70)],
                       "data": data}
        with self.assertRaisesRegex(ValueError, "occupied map cell|clearance"):
            probe.analyze_narrow_passage_path(
                metadata, unsafe_path, (2.8, -0.70), (4.8, -0.70))

        unknown = list(data)
        row = math.floor((-1.10-metadata["origin"][1])/metadata["resolution"])
        col = math.floor((4.8-metadata["origin"][0])/metadata["resolution"])
        unknown[row*metadata["width"]+col] = -1
        with self.assertRaisesRegex(ValueError, "known-free"):
            probe.validate_narrow_passage_goal(metadata, unknown, (2.8, -1.05))

    def test_narrow_plan_unknown_slampixel_needs_fresh_sensor_cleared_global_costmap(self):
        metadata, data = probe.map_values(
            grid(width=150, height=120, resolution=0.05, origin=(-1.0, -3.0)))

        def pose(x, y):
            return NS(header=NS(frame_id="map"), pose=NS(position=NS(x=x, y=y)))

        for row in range(metadata["height"]):
            y = metadata["origin"][1]+(row+0.5)*metadata["resolution"]
            if probe.NARROW_PASSAGE_LOW_Y <= y <= probe.NARROW_PASSAGE_HIGH_Y:
                continue
            for col in range(metadata["width"]):
                x = metadata["origin"][0]+(col+0.5)*metadata["resolution"]
                if abs(x-probe.NARROW_GATE_X) <= probe.NARROW_GATE_HALF_THICKNESS:
                    data[row*metadata["width"]+col] = 100
        plan = {"frame_id": "map", "poses": [pose(2.8, -1.05), pose(3.5, -1.10),
                 pose(4.8, -1.10)], "data": list(data)}
        x, y = 3.975, -1.125
        col = math.floor((x-metadata["origin"][0])/metadata["resolution"])
        row = math.floor((y-metadata["origin"][1])/metadata["resolution"])
        plan["data"][row*metadata["width"]+col] = -1
        costmap_data = list(data)
        snapshot = {"metadata": metadata, "data": costmap_data,
                    "stamp_ns": 9_900_000_000, "clock_ns_at_plan": 10_000_000_000,
                    "receive_age_at_plan_s": 0.10, "scan_age_at_plan_s": 0.12,
                    "publishers": ["global_costmap/global_costmap"]}
        result = probe.analyze_narrow_passage_path(
            metadata, plan, (2.8, -1.05), probe.NARROW_PASSAGE_GOAL,
            global_costmap=snapshot)
        self.assertEqual(result["plan_costmap_source"], "live_nav2_global_costmap")
        self.assertGreater(result["static_map_unknown_sample_count"], 0)
        self.assertGreater(result["global_costmap_receive_age_at_plan_s"], 0)

        inflated_costmap = dict(snapshot, data=list(costmap_data))
        inflated_costmap["data"][row*metadata["width"]+col] = 99
        inflated_result = probe.analyze_narrow_passage_path(
            metadata, plan, (2.8, -1.05), probe.NARROW_PASSAGE_GOAL,
            global_costmap=inflated_costmap)
        self.assertGreater(inflated_result["inflated_cost_plan_sample_count"], 0)

        unknown_costmap = dict(snapshot, data=list(costmap_data))
        unknown_costmap["data"][row*metadata["width"]+col] = -1
        with self.assertRaisesRegex(ValueError, "known-free planner costmap"):
            probe.analyze_narrow_passage_path(
                metadata, plan, (2.8, -1.05), probe.NARROW_PASSAGE_GOAL,
                global_costmap=unknown_costmap)

        stale_scan = dict(snapshot, scan_age_at_plan_s=3.0)
        with self.assertRaisesRegex(ValueError, "stale/unavailable"):
            probe.analyze_narrow_passage_path(
                metadata, plan, (2.8, -1.05), probe.NARROW_PASSAGE_GOAL,
                global_costmap=stale_scan)

    def test_narrow_passage_uses_a_later_fresh_replan_aligned_to_ground_truth(self):
        metadata, data = probe.map_values(
            grid(width=150, height=120, resolution=0.05, origin=(-1.0, -3.0)))

        def pose(x, y):
            return NS(header=NS(frame_id="map"), pose=NS(position=NS(x=x, y=y)))

        points = [pose(2.8, -1.05), pose(3.5, -1.10),
                  pose(4.0, -1.10), pose(4.8, -1.10)]
        costmap = {"metadata": metadata, "data": list(data),
                   "stamp_ns": 10_000_000_000, "clock_ns_at_plan": 10_050_000_000,
                   "receive_age_at_plan_s": 0.10, "scan_age_at_plan_s": 0.08,
                   "publishers": ["global_costmap/global_costmap"]}
        initial = {"frame_id": "map", "poses": points, "data": data,
                   "global_costmap": dict(costmap, receive_age_at_plan_s=3.0),
                   "time": 100.0,
                   "ground_truth_pose": {"time": 99.9, "pose": {
                       "position": [2.8, -1.05, 0.0]}}}
        valid = {"frame_id": "map", "poses": points, "data": data,
                 "global_costmap": costmap, "time": 101.0,
                 "ground_truth_pose": {"time": 100.9, "pose": {
                     "position": [2.8, -1.05, 0.0]}}}
        selected, summary, rejected = probe.select_narrow_passage_plan(
            [initial, valid], metadata, probe.NARROW_PASSAGE_GOAL)
        self.assertIs(selected, valid)
        self.assertEqual(summary["passage_crossings"][0]["x_m"], probe.NARROW_GATE_X)
        self.assertGreaterEqual(summary["minimum_robot_clearance_m"],
                                probe.MIN_ROBOT_CLEARANCE)
        self.assertEqual(len(rejected), 1)
        self.assertIn("stale/unavailable", rejected[0]["reason"])

        after_gate = dict(valid)
        after_gate["ground_truth_pose"] = {"time": 100.9, "pose": {
            "position": [4.1, -1.1, 0.0]}}
        with self.assertRaisesRegex(ValueError, "no fresh replanned passage path"):
            probe.select_narrow_passage_plan(
                [after_gate], metadata, probe.NARROW_PASSAGE_GOAL)

    def test_select_avoidance_plan_finds_earliest_matching_plan_among_replans(self):
        def pose(x, y):
            return NS(header=NS(frame_id="map"), pose=NS(position=NS(x=x, y=y)))

        full_plan = {"poses": [pose(0.50, 0.0), pose(1.5, -0.9), pose(2.85, -0.9)]}
        midway_replan = {"poses": [pose(1.50, -0.9), pose(2.85, -0.9)]}
        near_goal_replan = {"poses": [pose(2.80, -0.9), pose(2.85, -0.9)]}

        plans = [full_plan, midway_replan, near_goal_replan]
        selected = probe.select_avoidance_plan(plans, (0.48, -0.01), (2.85, -0.90))
        self.assertIs(selected, full_plan)

        # Fallback when no plan matches both endpoints
        fallback = probe.select_avoidance_plan([near_goal_replan], (0.48, -0.01), (2.85, -0.90))
        self.assertIs(fallback, near_goal_replan)

        with self.assertRaisesRegex(ValueError, "no global planner paths"):
            probe.select_avoidance_plan([], (0.48, -0.01), (2.85, -0.90))

    def test_executed_ground_truth_trajectory_measures_obstacle_clearance(self):
        def samples(points, step=0.1):
            result = []
            for start, end in zip(points, points[1:]):
                count = math.ceil(math.dist(start, end)/step)
                for index in range(count):
                    ratio = index/count
                    result.append({"time": float(len(result)), "pose": {
                        "position": (start[0]+(end[0]-start[0])*ratio,
                                     start[1]+(end[1]-start[1])*ratio, 0.1)}})
            end = points[-1]
            result.append({"time": float(len(result)), "pose": {
                "position": (end[0], end[1], 0.1)}})
            return result

        around_box = probe.analyze_ground_truth_trajectory(
            samples([(0.55, 0), (1.2, -0.9), (3.0, -0.9), (3.0, 0)]))
        self.assertGreater(around_box["minimum_robot_clearance_m"], 0.15)
        self.assertGreater(around_box["samples"], 3)
        through_box = probe.analyze_ground_truth_trajectory(samples([(0.55, 0), (3.0, 0)]))
        self.assertLess(through_box["minimum_robot_clearance_m"], 0.0)
        with self.assertRaisesRegex(ValueError, "too few"):
            probe.analyze_ground_truth_trajectory(samples([(0, 0), (0.1, 0)], step=1.0))


if __name__ == "__main__":
    unittest.main()
