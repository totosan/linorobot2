#!/usr/bin/env python3
"""Recompute independent passage clearance from persisted ground-truth points."""

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import verify_phase4_goal as probe  # noqa: E402

run_dir = Path(sys.argv[1])
report = json.loads((run_dir/"reachable-goal.json").read_text())
points = report["narrow_passage_ground_truth_xy_m"]
samples = [{"pose": {"position": (x, y, 0.0)}} for x, y in points]
trajectory = probe.analyze_ground_truth_trajectory(samples)
passage = probe.analyze_narrow_passage_trajectory(samples)
print("gt_points", len(points))
print("gt_min_clearance_m", trajectory["minimum_robot_clearance_m"],
      "at", trajectory.get("minimum_clearance_location_xy_m"))
print("gt_passage", {k: v for k, v in passage.items() if not isinstance(v, list)})
print("gt_crossings", passage.get("crossings") or passage.get("passage_crossings"))
