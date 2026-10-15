"""Static contract tests for the isolated Nav2 replay gate."""

from pathlib import Path
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Nav2ReplayGateTests(unittest.TestCase):
    @unittest.skipIf(importlib.util.find_spec("launch") is None,
                     "ROS 2 launch is available in the Humble runtime")
    def test_gate_composes_actual_common_and_fortress_parameters(self):
        import yaml

        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        command = gate.split(
            'python3 - "$NAV2_LAUNCH" "$NAV2_COMMON" "$NAV2_PROFILE" "$NAV2_PARAMS" <<\'PY\'\n',
            1)[1].split("\nPY", 1)[0]
        package = ROOT/"linorobot2_navigation"
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/"composed.yaml"
            subprocess.run(
                [sys.executable, "-", str(package/"launch/navigation.launch.py"),
                 str(package/"config/navigation_common.yaml"),
                 str(package/"config/navigation_fortress.yaml"), str(output)],
                input=command, text=True, check=True, capture_output=True)
            config = yaml.safe_load(output.read_text())
        for name in ("local_costmap", "global_costmap"):
            params = config[name][name]["ros__parameters"]
            self.assertEqual(params["obstacle_layer"]["plugin"],
                             "nav2_costmap_2d::ObstacleLayer")
            self.assertEqual(params["inflation_layer"]["plugin"],
                             "nav2_costmap_2d::InflationLayer")
            self.assertEqual(params["robot_radius"], 0.22)
            self.assertEqual(params["inflation_layer"]["inflation_radius"], 0.45)
        self.assertEqual(config["planner_server"]["ros__parameters"]["planner_plugins"],
                         ["GridBased"])

    def test_lifecycle_check_matches_humble_output_not_inactive_state(self):
        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        command = gate.split(
            'timeout 5 ros2 lifecycle get "$node" 2>/dev/null | ', 1)[1].split("; then", 1)[0]
        for value, active in (("active [3]", True), ("inactive [2]", False),
                              ("unconfigured [1]", False)):
            result = subprocess.run(["bash", "-c", command], input=value+"\n",
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode == 0, active)

    def test_costmap_sample_paths_do_not_require_uncreated_directories(self):
        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        self.assertIn('safe_name="${topic//\\//_}"', gate)
        self.assertNotIn('safe_name="${topic#/}"', gate)

    def test_replay_domain_and_nav2_consumer_are_explicit(self):
        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        self.assertIn('"$ROS_DOMAIN_ID" != "43"', gate)
        self.assertIn('export ROS_DOMAIN_ID=43', gate)
        self.assertIn('ros2 launch nav2_bringup navigation_launch.py', gate)
        self.assertIn('use_sim_time:=True', gate)
        self.assertIn('ros2 node list --no-daemon', gate)
        self.assertIn('planner_server', gate)
        self.assertIn('/global_costmap/costmap', gate)
        self.assertIn('/local_costmap/costmap', gate)

    def test_only_validated_inputs_are_played_and_command_path_is_disconnected(self):
        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        play_command = gate[gate.index('ros2 bag play'):gate.index('>\"$LOG_DIR/player.log\"')]
        for topic in ("/clock", "/tf", "/tf_static", "/scan", "/map", "/odom"):
            self.assertIn(topic, play_command)
        for topic in ("/cmd_vel", "/plan", "/global_costmap/costmap",
                      "/local_costmap/costmap"):
            self.assertNotIn(topic, play_command)
        self.assertIn('Subscription count: [1-9]', gate)
        self.assertIn('hardware_topics_or_devices', gate)
        self.assertIn('file hash mismatch', gate)

    def test_input_probes_precede_playback_and_run_concurrently(self):
        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        self.assertLess(
            gate.index("start_sample_probe /clock"),
            gate.index("ros2 bag play"))
        self.assertGreater(
            gate.index('wait_lifecycle_active "$node"'),
            gate.index("ros2 bag play"))
        self.assertGreater(
            gate.index(".nav2-subscribers.txt"),
            gate.index("SAMPLE_CHECK_FAILED=0"))
        self.assertIn(
            'timeout 15 ros2 topic echo --once "$@" "$topic" "$message_type" \\',
            gate)
        for invocation in (
                "start_sample_probe /clock rosgraph_msgs/msg/Clock",
                "start_sample_probe /tf tf2_msgs/msg/TFMessage",
                "start_sample_probe /tf_static tf2_msgs/msg/TFMessage",
                "start_sample_probe /scan sensor_msgs/msg/LaserScan",
                "start_sample_probe /map nav_msgs/msg/OccupancyGrid",
                "start_sample_probe /odom nav_msgs/msg/Odometry",
                "start_sample_probe /global_costmap/costmap nav_msgs/msg/OccupancyGrid",
                "start_sample_probe /local_costmap/costmap nav_msgs/msg/OccupancyGrid",
        ):
            self.assertIn(invocation, gate)
        self.assertIn('SAMPLE_PROBE_PIDS+=("$!")', gate)
        self.assertIn('PUBLISHER_WAIT_PIDS+=("$!")', gate)
        self.assertIn('"replay-report.json"', gate)

    def test_publisher_wait_retries_initial_topic_discovery_failure(self):
        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        wait_function = gate.split("wait_for_publisher() {\n", 1)[1].split(
            "\n}\n\ncheck_cmd_vel_unsubscribed()", 1)[0]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake_ros2 = root/"ros2"
            fake_ros2.write_text(
                "#!/usr/bin/env bash\n"
                "count=0\n"
                "[[ -f \"$FAKE_COUNT\" ]] && count=$(cat \"$FAKE_COUNT\")\n"
                "count=$((count + 1))\n"
                "printf '%s\\n' \"$count\" >\"$FAKE_COUNT\"\n"
                "if (( count == 1 )); then exit 1; fi\n"
                "if [[ \"$*\" == *\"--verbose\"* ]]; then\n"
                "  printf 'Publisher count: 1\\nEndpoint type: PUBLISHER\\n'\n"
                "else\n"
                "  printf 'Type: sensor_msgs/msg/LaserScan\\nPublisher count: 1\\n'\n"
                "fi\n")
            fake_ros2.chmod(0o755)
            command = (
                "set -euo pipefail\n"
                f"wait_for_publisher() {{\n{wait_function}\n}}\n"
                f"LOG_DIR={root!s}\n"
                "PLAYER_PID=$$\n"
                "wait_for_publisher /scan 1\n"
            )
            environment = {
                "PATH": f"{root}:{Path('/usr/bin')}:{Path('/bin')}",
                "FAKE_COUNT": str(root/"count"),
            }
            result = subprocess.run(
                ["bash", "-c", command], text=True, capture_output=True,
                env=environment, timeout=5)

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "1\n")

    def test_replay_report_records_samples_and_unknown_cmd_vel_truthfully(self):
        gate = (ROOT/"scripts/test_nav2_replay_gate.sh").read_text()
        command = gate.rsplit(
            'python3 - "$LOG_DIR" "$SEGMENT" "$MANIFEST" <<\'PY\'\n',
            1)[1].split("\nPY", 1)[0]
        topics = (
            "/clock", "/tf", "/tf_static", "/scan", "/map", "/odom",
            "/global_costmap/costmap", "/local_costmap/costmap",
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            log_dir = root/"run"
            log_dir.mkdir()
            segment = root/"segment-000"
            segment.mkdir()
            manifest = root/"segment-000.manifest.json"
            manifest.write_text('{"status":"PASS"}')
            for topic in topics:
                (log_dir/f"{topic.replace('/', '_')}.sample.txt").write_text("sample")
            (log_dir/"cmd-vel-ownership.txt").write_text(
                "Unknown topic '/cmd_vel'\n")

            subprocess.run(
                [sys.executable, "-", str(log_dir), str(segment), str(manifest)],
                input=command, text=True, check=True, capture_output=True)
            report = json.loads((log_dir/"replay-report.json").read_text())

        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["scope"], "NAV2_SENSOR_INPUT_SEGMENT_V1")
        self.assertEqual(report["cmd_vel_subscriber_count"], None)
        self.assertEqual(report["cmd_vel_observation"], "topic_not_discovered")
        self.assertEqual(set(report["samples"]), set(topics))


if __name__ == "__main__":
    unittest.main()
