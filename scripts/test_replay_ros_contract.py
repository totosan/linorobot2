"""Guard the documented, non-executable Domain-43 replay contract."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"linorobot2_base"))
from linorobot2_base.domain_profile import resolve_ros_domain  # noqa: E402


class ReplayRosContractTests(unittest.TestCase):
    def test_reserved_replay_domain_and_clock_mode_are_explicit(self):
        contract = (ROOT/"docs/REPLAY_ROS_CONTRACT.md").read_text()
        self.assertEqual(resolve_ros_domain("replay", "43"), "43")
        self.assertIn("Domain 43", contract)
        self.assertIn('resolve_ros_domain("replay", "43")', contract)
        self.assertIn("Do not add `--clock`", contract)
        self.assertIn("`use_sim_time=true`", contract)
        self.assertIn("`clock_domain=ros_sim`", contract)

    def test_replay_allowlist_excludes_commands_and_derived_outputs(self):
        contract = (ROOT/"docs/REPLAY_ROS_CONTRACT.md").read_text()
        self.assertIn("/clock /camera/image_raw /camera/camera_info", contract)
        self.assertIn("Exclude `/cmd_vel`", contract)
        self.assertIn("`/vision/result`", contract)
        self.assertIn("implemented and runtime-proven", contract)
        self.assertIn("replay-20261009-145320-19HHgD", contract)
        self.assertIn("rosbag2_player", contract)


if __name__ == "__main__":
    unittest.main()
