#!/usr/bin/env python3
"""Read-only summary of a persisted G4 run directory (no ROS, no simulator)."""

import json
from pathlib import Path
import sys


def _scalars(item):
    return {key: (round(value, 4) if isinstance(value, float) else value)
            for key, value in item.items() if not isinstance(value, (list, dict))}


def main(argv):
    run_dir = Path(argv[1])
    report = json.loads((run_dir/"reachable-goal.json").read_text())
    print("status", report["status"], "pass", report["pass"],
          "elapsed_s", round(report["elapsed_seconds"], 1), "errors", report["errors"])
    for attempt in report["goal_attempts"]:
        stop = attempt.get("stop_proof") or {}
        print(attempt["goal"], attempt["nav2_action_status"],
              "gt_distance_m", attempt.get("ground_truth_distance_to_goal_m"),
              "zero_after_result", attempt.get("zero_command_count_after_result"),
              "stillness", stop.get("post_result_stillness_samples"),
              "drift_m", stop.get("post_result_max_drift_m"))
    for key in sorted(report):
        value = report[key]
        if not isinstance(value, dict):
            continue
        if any(tag in key for tag in ("plan_evidence", "trajectory_evidence", "passage")):
            print(key, _scalars(value))
            if "passage_crossings" in value:
                print("  crossings", value["passage_crossings"])
        if key.endswith("plan_costmap"):
            print(key, {k: value.get(k) for k in (
                "receive_age_at_plan_s", "scan_age_at_plan_s", "publishers")})
    mapping = json.loads((run_dir/"slam-mapping.json").read_text())
    print("mapping_status", mapping.get("status"),
          "mapping_min_clearance_m", mapping.get("minimum_obstacle_clearance_m"))
    info = (run_dir/"rosbag-info.txt").read_text()
    messages = info.split("Messages:")[1].split("\n")[0].strip() if "Messages:" in info else "?"
    size = sum(f.stat().st_size for f in (run_dir/"rosbag").glob("*.db3"))//1048576
    print("rosbag_messages", messages, "rosbag_mib", size)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
