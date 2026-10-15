"""Static regression tests for the isolated G5 off/mock SIL harness."""

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Phase5GateTests(unittest.TestCase):
    def test_gate_runs_off_then_mock_on_one_map_and_compares_reports(self):
        text = (ROOT/"scripts/test_phase5_gate.sh").read_text()
        self.assertIn("run_mode off", text)
        self.assertIn("run_mode mock", text)
        self.assertLess(text.index("run_mode off"), text.index("run_mode mock"))
        self.assertIn('G5_MAP_YAML', text)
        self.assertIn('ground_truth_used_for_navigation_map") is False', text)
        self.assertIn('common_params_file:="$COMMON_PROFILE"', text)
        self.assertIn('verify_g5_equivalence.py', text)
        self.assertIn('trap cleanup EXIT', text)

    def test_gate_uses_fixture_mock_and_records_status_without_image_payloads(self):
        text = (ROOT/"scripts/test_phase5_gate.sh").read_text()
        self.assertIn('fixture_path:=$VISION_FIXTURE', text)
        self.assertIn('verify_vision_ros_transport.py', text)
        self.assertIn('/vision/result /vision/status', text)
        self.assertNotIn('/camera/image_raw /camera/camera_info /vision/result', text)
        self.assertIn('ROS_DOMAIN_ID=42', text)
        self.assertIn('ros_profile:=sim ros_domain_id:=42', text)

    def test_transport_probe_and_adapter_cannot_publish_robot_commands(self):
        node = (ROOT/"linorobot2_vision/python/linorobot2_vision"
                /"ros_node.py").read_text()
        probe = (ROOT/"scripts/verify_vision_ros_transport.py").read_text()
        self.assertNotIn("cmd_vel", node)
        self.assertIn('get_publishers_info_by_topic("/cmd_vel")', probe)
        self.assertIn('"linorobot2_vision_adapter"', probe)

    def test_gate_preflights_the_sim_domain_guard_before_runtime(self):
        text = (ROOT/"scripts/test_phase5_gate.sh").read_text()
        guard = text.index("from linorobot2_base.domain_profile import resolve_ros_domain")
        bag = text.index("start_bag()")
        self.assertLess(guard, bag)
        self.assertIn('resolve_ros_domain("sim", "42")', text)

    def test_gate_preflights_all_ament_packages_before_runtime(self):
        text = (ROOT/"scripts/test_phase5_gate.sh").read_text()
        package_check = text.index('ros2 pkg prefix "$package"')
        first_runtime = text.index("LOG_ROOT=\"$(mktemp")
        self.assertLess(package_check, first_runtime)
        for package in ("linorobot2_base", "linorobot2_description",
                        "linorobot2_gazebo", "linorobot2_navigation",
                        "linorobot2_vision"):
            self.assertIn(package, text)
        self.assertIn('env ROS_DOMAIN_ID="$ROS_DOMAIN_ID"', text)
        self.assertIn('IGN_PARTITION="$IGN_PARTITION" python3 -u', text)

    def test_gate_uses_canonical_g4_partition_prefix(self):
        text = (ROOT/"scripts/test_phase5_gate.sh").read_text()
        self.assertIn('G5_PARTITION="linobot2_g4_${UID}_g5_', text)
        self.assertIn('[[ "$IGN_PARTITION" != linobot2_g4_*', text)
        self.assertNotIn('G5_PARTITION="linorobot2_g4_g5_', text)


if __name__ == "__main__":
    unittest.main()
