"""Static regression tests for the Domain-43 replay gate."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"linorobot2_base"))
from linorobot2_base.domain_profile import resolve_ros_domain  # noqa: E402


class ReplayGateTests(unittest.TestCase):
    def test_gate_enforces_replay_domain_and_segment_provenance(self):
        text = (ROOT/"scripts/test_replay_gate.sh").read_text()
        self.assertIn('resolve_ros_domain("replay", "43")', text)
        self.assertIn('export ROS_DOMAIN_ID=43', text)
        self.assertIn('ROS_DOMAIN_ID" != "43"', text)
        self.assertIn('manifest is not a validated ros_sim replay segment', text)
        self.assertIn('segment files do not match manifest hashes', text)

    def test_gate_replays_exactly_the_contract_allowlist_without_clock_generator(self):
        text = (ROOT/"scripts/test_replay_gate.sh").read_text()
        self.assertIn('--topics /clock /camera/image_raw /camera/camera_info', text)
        self.assertIn('--disable-keyboard-controls', text)
        play_command = text[text.index('ros2 bag play'):text.index('>"$LOG_DIR/player.log"')]
        self.assertNotIn('--clock', play_command)
        self.assertIn('Publisher count: ', text)
        self.assertIn('exactly one /clock publisher', text)
        for topic in ("/cmd_vel", "/plan", "/global_costmap"):
            self.assertNotIn(topic, text)

    def test_gate_starts_one_adapter_mode_and_cleans_up(self):
        text = (ROOT/"scripts/test_replay_gate.sh").read_text()
        self.assertIn('--mode=off', text)
        self.assertIn('--mode=mock', text)
        self.assertIn('--domain 43', text)
        self.assertIn('clock_domain:=ros_sim', text)
        self.assertIn('use_sim_time:=true', text)
        self.assertIn('trap cleanup EXIT', text)
        self.assertIn('stop_owned "$PLAYER_PID"', text)

    def test_replay_domain_resolves_and_segment_topics_are_pinned(self):
        self.assertEqual(resolve_ros_domain("replay", "43"), "43")
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "create_replay_segment", ROOT/"scripts/create_replay_segment.py")
        creator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(creator)
        self.assertEqual(set(creator.TOPIC_TYPES),
                         {"/clock", "/camera/image_raw", "/camera/camera_info"})
        self.assertEqual(creator.FRAME_ID, "camera_link")


if __name__ == "__main__":
    unittest.main()
