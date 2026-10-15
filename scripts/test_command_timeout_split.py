"""ROS-free checks for the shared command-timeout package/launch contract."""

from pathlib import Path
import os
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
SRC = ROOT


class CommandTimeoutSplitTests(unittest.TestCase):
    def test_timeout_is_installed_from_base_package(self):
        base = SRC / "linorobot2_base"
        gazebo = SRC / "linorobot2_gazebo"
        self.assertTrue((base / "scripts/command_timeout.py").is_file())
        self.assertTrue(os.access(base / "scripts/command_timeout.py", os.X_OK))
        self.assertIn("scripts/command_timeout.py", (base / "CMakeLists.txt").read_text())
        self.assertNotIn("scripts/command_timeout.py", (gazebo / "CMakeLists.txt").read_text())
        self.assertFalse((gazebo / "scripts/command_timeout.py").exists())

        dependencies = {
            node.text for node in ET.parse(base / "package.xml").getroot().findall("exec_depend")
        }
        self.assertTrue({"rclpy", "geometry_msgs", "std_msgs"}.issubset(dependencies))

    def test_simulation_launches_use_the_shared_wall_clock_node(self):
        gazebo = SRC / "linorobot2_gazebo/launch"
        for filename in ("fortress.launch.py", "gazebo.launch.py"):
            with self.subTest(filename=filename):
                launch = (gazebo / filename).read_text()
                self.assertIn("package='linorobot2_base'", launch)
                self.assertIn("executable='command_timeout.py'", launch)
                self.assertIn("parameters=[{'use_sim_time': False}]", launch)

        dependencies = {
            node.text for node in ET.parse(SRC / "linorobot2_gazebo/package.xml")
            .getroot().findall("exec_depend")
        }
        self.assertIn("linorobot2_base", dependencies)

    def test_hardware_bringup_exposes_watchdog_only_as_explicit_opt_in(self):
        bringup = SRC / "linorobot2_bringup"
        launch = (bringup / "launch/bringup.launch.py").read_text()
        argument = re.search(
            r"DeclareLaunchArgument\(\s*name='enable_command_timeout',\s*default_value='false'",
            launch,
        )
        self.assertIsNotNone(argument)
        self.assertIn("IfCondition(LaunchConfiguration('enable_command_timeout'))", launch)
        self.assertIn("package='linorobot2_base'", launch)
        self.assertIn("parameters=[{'use_sim_time': False}]", launch)

        dependencies = {
            node.text for node in ET.parse(bringup / "package.xml").getroot()
            .findall("exec_depend")
        }
        self.assertIn("linorobot2_base", dependencies)


if __name__ == "__main__":
    unittest.main()