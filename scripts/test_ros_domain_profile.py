"""ROS-free tests for profile/domain validation and launch ordering."""

import ast
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).parents[1]
BASE_PACKAGE = ROOT / "linorobot2_base"
sys.path.insert(0, str(BASE_PACKAGE))

from linorobot2_base.domain_profile import (  # noqa: E402
    ROS_DOMAIN_BY_PROFILE,
    resolve_ros_domain,
)


class DomainContractTests(unittest.TestCase):
    def test_fixed_profile_domain_pairs(self):
        self.assertEqual(ROS_DOMAIN_BY_PROFILE,
                         {"sim": "42", "replay": "43", "hardware": "44"})
        for profile, domain in ROS_DOMAIN_BY_PROFILE.items():
            with self.subTest(profile=profile):
                self.assertEqual(resolve_ros_domain(profile), domain)
                self.assertEqual(resolve_ros_domain(profile, domain, domain), domain)

    def test_launch_profile_domain_and_inherited_domain_must_match(self):
        cases = (
            ("sim", "44", None, None),
            ("hardware", "42", None, None),
            ("replay", "043", None, None),
            ("sim", "42", "44", None),
            ("hardware", "44", None, "sim"),
            ("unknown", None, None, None),
        )
        for profile, requested, inherited, launch_profile in cases:
            with self.subTest(profile=profile, requested=requested,
                              inherited=inherited, launch_profile=launch_profile):
                with self.assertRaises(ValueError):
                    resolve_ros_domain(profile, requested, inherited, launch_profile)

    def test_launches_guard_before_starting_ros_or_simulator_processes(self):
        launch_contracts = (
            (ROOT / "linorobot2_gazebo/launch/fortress.launch.py",
             "sim", "42"),
            (ROOT / "linorobot2_gazebo/launch/gazebo.launch.py",
             "sim", "42"),
            (ROOT / "linorobot2_bringup/launch/bringup.launch.py",
             "hardware", "44"),
        )
        for path, profile, domain in launch_contracts:
            with self.subTest(path=path.name):
                tree = ast.parse(path.read_text(), filename=str(path))
                source = path.read_text()
                self.assertIn("from linorobot2_base.domain_profile import resolve_ros_domain",
                              source)
                self.assertIn(f"expected_profile='{profile}'", source)
                self.assertIn(f"default_value='{profile}'", source)
                self.assertIn(f"default_value='{domain}'", source)
                self.assertIn("SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=domain_id)",
                              source)

                launch_function = next(
                    node for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "generate_launch_description"
                )
                launch_return = next(
                    node for node in ast.walk(launch_function)
                    if isinstance(node, ast.Return)
                    and isinstance(node.value, ast.Call)
                    and isinstance(node.value.func, ast.Name)
                    and node.value.func.id == "LaunchDescription"
                )
                actions = launch_return.value.args[0].elts
                guard_index = next(
                    index for index, action in enumerate(actions)
                    if isinstance(action, ast.Call)
                    and isinstance(action.func, ast.Name)
                    and action.func.id == "OpaqueFunction"
                )
                process_index = next(
                    index for index, action in enumerate(actions)
                    if ((isinstance(action, ast.Call)
                         and isinstance(action.func, ast.Name)
                         and action.func.id in {"Node", "ExecuteProcess"})
                        or (isinstance(action, ast.Name) and action.id == "server"))
                )
                self.assertLess(guard_index, process_index)


if __name__ == "__main__":
    unittest.main()