"""ROS-free regression checks for the manual Jetson container profile."""

from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).parents[1]


class JetsonDeploymentProfileTests(unittest.TestCase):
    def test_compose_maps_only_role_specific_serial_devices(self):
        compose = (ROOT / "docker/docker-compose.jetson.yaml").read_text()
        self.assertIn("/dev/ttyESP32}:/dev/ttyUSB0", compose)
        self.assertIn("/dev/rplidar}:/dev/ttyUSB1", compose)
        self.assertNotIn("privileged:", compose)
        self.assertNotIn("/dev:/dev", compose)
        self.assertNotIn("restart:", compose)
        self.assertIn("ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-44}", compose)
        self.assertIn("webcam_enabled:=false", compose)
        self.assertIn("ros_profile:=hardware", compose)
        builder = compose.split("  builder:\n", 1)[1].split("\n  robot:", 1)[0]
        self.assertIn('profiles: ["build"]', builder)
        self.assertIn("network_mode: none", builder)
        self.assertIn("read_only: true", builder)
        self.assertIn("ROS_HOME: /tmp/ros", builder)
        self.assertIn("ROS_LOG_DIR: /tmp/ros/log", builder)
        self.assertIn("colcon build --packages-select linorobot2_base linorobot2_bringup", builder)
        self.assertIn("resolve_ros_domain('hardware', '44') == '44'", builder)
        self.assertIn("install/linorobot2_bringup/share/linorobot2_bringup/launch/bringup.launch.py", builder)
        self.assertIn("validate_lidar_monitor_launch.py", builder)
        self.assertNotIn("devices:", builder)

    def test_manual_preflight_fails_closed_on_missing_or_duplicate_hardware(self):
        launcher = (ROOT / "docker/start_jetson_container.sh").read_text()
        self.assertIn('[[ ! -c "${device}" ]]', launcher)
        self.assertIn('[[ "${esp32_real}" == "${lidar_real}" ]]', launcher)
        self.assertIn("install/setup.bash", launcher)
        self.assertIn("docker compose", launcher)
        self.assertNotIn("chmod", launcher)

    def test_startup_wrapper_rejects_missing_device_before_docker(self):
        launcher = ROOT / "docker/start_jetson_container.sh"
        with tempfile.TemporaryDirectory() as temp_dir:
            env_file = Path(temp_dir) / "jetson.env"
            env_file.write_text(
                f"JETSON_WORKSPACE={temp_dir}\n"
                "JETSON_ESP32_DEVICE=/device-that-does-not-exist\n"
                "JETSON_LIDAR_DEVICE=/another-missing-device\n"
            )
            result = subprocess.run(
                ["bash", str(launcher)],
                env={**os.environ, "JETSON_ENV_FILE": str(env_file)},
                capture_output=True,
                text=True,
                check=False,
            )
        self.assertEqual(result.returncode, 3)
        self.assertIn("Required serial character device is unavailable", result.stderr)

    def test_ros_launch_propagates_micro_ros_and_lidar_device_settings(self):
        bringup = (ROOT / "linorobot2_bringup/launch/bringup.launch.py").read_text()
        default_robot = (ROOT / "linorobot2_bringup/launch/default_robot.launch.py").read_text()
        sensors = (ROOT / "linorobot2_bringup/launch/sensors.launch.py").read_text()
        lasers = (ROOT / "linorobot2_bringup/launch/lasers.launch.py").read_text()
        extra = (ROOT / "linorobot2_bringup/launch/extra.launch.py").read_text()
        lidar_monitor = (ROOT / "linorobot2_bringup/scripts/lidar_monitor.py").read_text()
        bringup_cmake = (ROOT / "linorobot2_bringup/CMakeLists.txt").read_text()

        for argument in ("micro_ros_transport", "micro_ros_port", "micro_ros_baudrate",
                         "base_serial_port", "lidar_serial_port"):
            with self.subTest(argument=argument):
                self.assertIn(f"'{argument}': LaunchConfiguration(\"{argument}\")", bringup)
        self.assertIn("LaunchConfiguration(\"micro_ros_baudrate\")", default_robot)
        self.assertIn("LaunchConfiguration('micro_ros_port')", default_robot)
        self.assertIn("'lidar_serial_port': LaunchConfiguration('lidar_serial_port')", default_robot)
        self.assertIn("'lidar_serial_port': LaunchConfiguration('lidar_serial_port')", sensors)
        self.assertIn("'lidar_serial_port': LaunchConfiguration(\"lidar_serial_port\")", bringup)
        self.assertIn("LaunchConfiguration('lidar_serial_port')", extra)
        self.assertIn("'serial_port': LaunchConfiguration('lidar_serial_port')", lasers)
        self.assertIn("'port': LaunchConfiguration('lidar_serial_port')", lasers)
        self.assertEqual(lasers.count("'/dev/ttyUSB1'"), 1)  # argument default only
        self.assertIn("name='lidar_required'", bringup)
        self.assertIn("default_value='true'", bringup)
        self.assertIn("RegisterEventHandler", bringup)
        self.assertIn("OnProcessExit", bringup)
        self.assertIn("Shutdown(", bringup)
        self.assertIn("scripts/lidar_monitor.py", bringup_cmake)
        self.assertIn("scripts/lidar_monitor_contract.py", bringup_cmake)
        for dangerous_operation in ("killall", "os.killpg", "os._exit", "subprocess.run"):
            with self.subTest(operation=dangerous_operation):
                self.assertNotIn(dangerous_operation, lidar_monitor)


if __name__ == "__main__":
    unittest.main()
