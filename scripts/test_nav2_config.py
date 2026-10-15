#!/usr/bin/env python3
"""ROS-free regression tests for shared/profile Nav2 parameter composition."""

import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "linorobot2_navigation"
INSTALLED_CONFIG = (Path.home()/"linorobot2_sim/install/linorobot2_navigation"
                    /"share/linorobot2_navigation/config")
try:
    import yaml
except ImportError:
    yaml = None

try:
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import perform_substitutions
    launch_spec = importlib.util.spec_from_file_location(
        "navigation_launch_under_test", PACKAGE / "launch/navigation.launch.py")
    navigation_launch = importlib.util.module_from_spec(launch_spec)
    launch_spec.loader.exec_module(navigation_launch)
except ImportError:
    navigation_launch = None


class Nav2ConfigMergeTests(unittest.TestCase):
    @unittest.skipIf(navigation_launch is None,
                     "ROS 2 launch packages are available in the Humble runtime")
    def test_recursive_merge_preserves_profile_precedence_and_replaces_lists(self):
        common = {
            "node": {"ros__parameters": {
                "shared": 1, "nested": {"shared": True, "list": [1, 2]}}}}
        profile = {
            "node": {"ros__parameters": {
                "profile": 2, "nested": {"profile": False, "list": [3]}}}}
        self.assertEqual(navigation_launch._merge_parameter_trees(common, profile), {
            "node": {"ros__parameters": {
                "shared": 1, "profile": 2,
                "nested": {"shared": True, "profile": False, "list": [3]}}}})

    @unittest.skipIf(navigation_launch is None,
                     "ROS 2 launch packages are available in the Humble runtime")
    def test_parameter_roots_must_be_mappings(self):
        for common, profile in ((None, {}), ({}, None), ([], {})):
            with self.subTest(common=common, profile=profile), self.assertRaises(TypeError):
                navigation_launch._merge_parameter_trees(common, profile)

    @unittest.skipIf(yaml is None or navigation_launch is None,
                     "PyYAML and ROS 2 launch are available in the Humble runtime")
    def test_shared_contract_matches_both_profiles_and_preserves_profile_semantics(self):
        common_path = PACKAGE / "config/navigation_common.yaml"
        profiles = {
            "hardware": PACKAGE / "config/navigation.yaml",
            "fortress": PACKAGE / "config/navigation_fortress.yaml",
        }
        common = yaml.safe_load(common_path.read_text())
        with tempfile.TemporaryDirectory() as temporary_directory:
            merged_profiles = {}
            for name, profile_path in profiles.items():
                output = Path(temporary_directory) / f"{name}.yaml"
                navigation_launch._compose_parameter_file(common_path, profile_path, output)
                merged_profiles[name] = yaml.safe_load(output.read_text())

                # Shared fields belong to the common source, not duplicated profiles.
                profile = yaml.safe_load(profile_path.read_text())
                self.assertNotIn("planner_plugins",
                                 profile["planner_server"]["ros__parameters"])
                for costmap in ("global_costmap", "local_costmap"):
                    params = profile[costmap][costmap]["ros__parameters"]
                    self.assertNotIn("robot_radius", params)
                    self.assertNotIn("resolution", params)
                    self.assertNotIn("plugin", params["obstacle_layer"])
                    self.assertNotIn("enabled", params["obstacle_layer"])
                    self.assertNotIn("plugin", params["inflation_layer"])

            hardware = merged_profiles["hardware"]
            fortress = merged_profiles["fortress"]
            self.assertEqual(common["planner_server"]["ros__parameters"]
                             ["planner_plugins"], ["GridBased"])
            for config in merged_profiles.values():
                self.assertEqual(config["planner_server"]["ros__parameters"]
                                 ["planner_plugins"], ["GridBased"])
                for costmap in ("global_costmap", "local_costmap"):
                    params = config[costmap][costmap]["ros__parameters"]
                    self.assertEqual(params["robot_radius"], 0.22)
                    self.assertEqual(params["resolution"], 0.05)
                    self.assertEqual(params["obstacle_layer"]["plugin"],
                                     "nav2_costmap_2d::ObstacleLayer")
                    self.assertTrue(params["obstacle_layer"]["enabled"])
                    self.assertEqual(params["inflation_layer"]["plugin"],
                                     "nav2_costmap_2d::InflationLayer")

            global_common = common["global_costmap"]["global_costmap"]["ros__parameters"]
            self.assertTrue(global_common["always_send_full_costmap"])
            self.assertEqual(global_common["static_layer"]["plugin"],
                             "nav2_costmap_2d::StaticLayer")
            self.assertTrue(global_common["static_layer"]["map_subscribe_transient_local"])
            self.assertTrue(global_common["static_layer"]["subscribe_to_updates"])

            # Time, frames, planner/unknown policy, speed and inflation remain profile-owned.
            self.assertFalse(hardware["amcl"]["ros__parameters"]["use_sim_time"])
            self.assertTrue(fortress["amcl"]["ros__parameters"]["use_sim_time"])
            self.assertEqual(hardware["bt_navigator"]["ros__parameters"]["robot_base_frame"],
                             "base_link")
            self.assertEqual(fortress["bt_navigator"]["ros__parameters"]["robot_base_frame"],
                             "base_footprint")
            self.assertTrue(hardware["planner_server"]["ros__parameters"]
                            ["GridBased"]["allow_unknown"])
            self.assertFalse(fortress["planner_server"]["ros__parameters"]
                             ["GridBased"]["allow_unknown"])
            self.assertEqual(hardware["planner_server"]["ros__parameters"]
                             ["GridBased"]["plugin"],
                             "nav2_navfn_planner/NavfnPlanner")
            self.assertEqual(fortress["planner_server"]["ros__parameters"]
                             ["GridBased"]["plugin"],
                             "nav2_smac_planner/SmacPlanner2D")
            self.assertEqual(hardware["controller_server"]["ros__parameters"]
                             ["FollowPath"]["max_vel_x"], 0.4)
            self.assertEqual(fortress["controller_server"]["ros__parameters"]
                             ["FollowPath"]["max_vel_x"], 0.08)
            self.assertEqual(hardware["global_costmap"]["global_costmap"]
                             ["ros__parameters"]["inflation_layer"]["inflation_radius"], 0.55)
            self.assertEqual(fortress["global_costmap"]["global_costmap"]
                             ["ros__parameters"]["inflation_layer"]["inflation_radius"], 0.45)

    @unittest.skipIf(yaml is None or navigation_launch is None,
                     "PyYAML and ROS 2 launch are available in the Humble runtime")
    def test_default_sim_profile_keeps_its_own_values_when_composed(self):
        common_path = PACKAGE / "config/navigation_common.yaml"
        profile_path = PACKAGE / "config/navigation_sim.yaml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "sim.yaml"
            navigation_launch._compose_parameter_file(common_path, profile_path, output)
            result = yaml.safe_load(output.read_text())
        profile = yaml.safe_load(profile_path.read_text())
        for costmap in ("global_costmap", "local_costmap"):
            node_params = result[costmap][costmap]["ros__parameters"]
            profile_params = profile[costmap][costmap]["ros__parameters"]
            self.assertEqual(node_params["robot_radius"], profile_params["robot_radius"])
            self.assertEqual(node_params["resolution"], profile_params["resolution"])

    @unittest.skipIf(navigation_launch is None,
                     "ROS 2 launch packages are available in the Humble runtime")
    def test_live_launch_does_not_load_a_sibling_helper_by_path(self):
        source = (PACKAGE / "launch/navigation.launch.py").read_text()
        self.assertNotIn("spec_from_file_location", source)
        self.assertNotIn("nav2_config.py", source)

    @unittest.skipIf(navigation_launch is None,
                     "ROS 2 launch packages are available in the Humble runtime")
    def test_common_parameter_default_is_an_existing_absolute_share_path(self):
        description = navigation_launch.generate_launch_description()
        common_argument = next(
            action for action in description.entities
            if getattr(action, "name", None) == "common_params_file")
        default = perform_substitutions(LaunchContext(), common_argument.default_value)
        self.assertTrue(Path(default).is_absolute())
        self.assertEqual(Path(default).name, "navigation_common.yaml")
        self.assertTrue(Path(default).is_file())

    def test_g4_harness_passes_the_common_profile_explicitly(self):
        gate = (ROOT / "scripts/test_phase4_gate.sh").read_text()
        self.assertIn("local common_params=", gate)
        self.assertIn('common_params_file:="$common_params"', gate)

    @unittest.skipIf(navigation_launch is None,
                     "ROS 2 launch packages are available in the Humble runtime")
    def test_nav2_callback_composes_profile_without_starting_nodes(self):
        context = LaunchContext()
        context.launch_configurations.update({
            "sim": "true",
            "sim_params_file": str(PACKAGE/"config/navigation_fortress.yaml"),
            "common_params_file": str(PACKAGE/"config/navigation_common.yaml"),
            "map": "/tmp/nonexistent-test-map.yaml",
        })
        actions = navigation_launch._launch_nav2(context)
        self.assertEqual(len(actions), 1)
        self.assertIsInstance(actions[0], IncludeLaunchDescription)

    @unittest.skipUnless(
        INSTALLED_CONFIG.is_dir() and navigation_launch is not None,
        "installed Humble navigation package and launch dependencies are not present")
    def test_nav2_callback_loads_installed_linux_profile_files(self):
        context = LaunchContext()
        context.launch_configurations.update({
            "sim": "true",
            "sim_params_file": str(INSTALLED_CONFIG/"navigation_fortress.yaml"),
            "common_params_file": str(INSTALLED_CONFIG/"navigation_common.yaml"),
            "map": "/tmp/nonexistent-test-map.yaml",
        })
        actions = navigation_launch._launch_nav2(context)
        self.assertEqual(len(actions), 1)
        self.assertIsInstance(actions[0], IncludeLaunchDescription)


if __name__ == "__main__":
    unittest.main()
