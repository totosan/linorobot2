#!/usr/bin/env python3
"""Read-only postmortem of a G4 Nav2 plan and its nearest recorded costmaps."""

import argparse
from array import array
import json
import math
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import verify_phase4_goal as probe


def message_stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec*1_000_000_000+stamp.nanosec


def nearest(items, bag_time_ns):
    if not items:
        return None
    return min(items, key=lambda item: abs(item["bag_time_ns"]-bag_time_ns))


def nearest_preceding(items, bag_time_ns):
    candidates = [item for item in items if item["bag_time_ns"] <= bag_time_ns]
    return max(candidates, key=lambda item: item["bag_time_ns"]) if candidates else None


def path_clearance(metadata, path):
    points = [(float(item.pose.position.x), float(item.pose.position.y))
              for item in path.poses]
    minimum = math.inf
    nearest_point = None
    static_unknown = []
    for start, end in zip(points, points[1:]):
        steps = max(1, math.ceil(math.dist(start, end)/0.025))
        for index in range(steps+1):
            ratio = index/steps
            x = start[0]+(end[0]-start[0])*ratio
            y = start[1]+(end[1]-start[1])*ratio
            clearance = probe.obstacle_clearance(x, y)
            if clearance < minimum:
                minimum = clearance
                nearest_point = [x, y]
            occupancy = probe.occupancy_at(metadata, metadata["data"], x, y)
            if occupancy is None or occupancy < 0:
                if len(static_unknown) < 100:
                    static_unknown.append({"x_m": x, "y_m": y, "occupancy": occupancy})
    return minimum, nearest_point, static_unknown


def passage_cross_section(static_metadata, costmap_metadata, gate_x):
    low = probe.NARROW_PASSAGE_LOW_Y+probe.ROBOT_RADIUS
    high = probe.NARROW_PASSAGE_HIGH_Y-probe.ROBOT_RADIUS
    count = max(1, math.ceil((high-low)/0.025))
    return [{"x_m": gate_x,
             "y_m": low+(high-low)*index/count,
             "static_map": probe.occupancy_at(
                 static_metadata, static_metadata["data"], gate_x, low+(high-low)*index/count),
             "global_costmap": probe.occupancy_at(
                 costmap_metadata, costmap_metadata["data"], gate_x, low+(high-low)*index/count)}
            for index in range(count+1)]


def costmap_timeline(costmaps, plan, sample_points, window_s=3.0):
    snapshots = []
    plan_record_time = plan["bag_time_ns"]
    for item in costmaps:
        record_delta = (item["bag_time_ns"]-plan_record_time)/1e9
        if abs(record_delta) > window_s:
            continue
        values = [{"x_m": point[0], "y_m": point[1],
                   "costmap": probe.occupancy_at(
                       item["metadata"], item["metadata"]["data"], *point)}
                  for point in sample_points]
        snapshots.append({"record_delta_to_plan_s": record_delta,
                          "header_stamp_delta_to_plan_s": (
                              item["stamp_ns"]-plan["stamp_ns"])/1e9,
                          "values": values})
    return snapshots


def costmap_timeline_after_plan(costmaps, plan, sample_points, duration_s=180.0):
    snapshots = []
    plan_record_time = plan["bag_time_ns"]
    for item in costmaps:
        record_delta = (item["bag_time_ns"]-plan_record_time)/1e9
        if not 0.0 <= record_delta <= duration_s:
            continue
        snapshots.append({"record_delta_to_plan_s": record_delta,
                          "header_stamp_delta_to_plan_s": (
                              item["stamp_ns"]-plan["stamp_ns"])/1e9,
                          "values": [{"x_m": point[0], "y_m": point[1],
                                      "costmap": probe.occupancy_at(
                                          item["metadata"], item["metadata"]["data"], *point)}
                                     for point in sample_points]})
    return snapshots


def summarize_plan_costmap(plan_item, costmap_item, static_metadata):
    path = plan_item["message"]
    points = [(float(pose.pose.position.x), float(pose.pose.position.y))
              for pose in path.poses]
    costmap = costmap_item["metadata"]
    unknown_samples, lethal_samples = 0, 0
    clearance = math.inf
    crossings = probe.passage_crossings(points)
    for start, end in zip(points, points[1:]):
        steps = max(1, math.ceil(math.dist(start, end)/0.025))
        for index in range(steps+1):
            ratio = index/steps
            x = start[0]+(end[0]-start[0])*ratio
            y = start[1]+(end[1]-start[1])*ratio
            cell = probe.occupancy_at(costmap, costmap["data"], x, y)
            unknown_samples += cell is None or cell < 0
            lethal_samples += cell == 100
            clearance = min(clearance, probe.obstacle_clearance(x, y))
    return {"plan_record_time_ns": plan_item["bag_time_ns"],
            "plan_stamp_ns": plan_item["stamp_ns"],
            "costmap_record_delta_to_plan_s": (
                costmap_item["bag_time_ns"]-plan_item["bag_time_ns"])/1e9,
            "costmap_stamp_delta_to_plan_s": (
                costmap_item["stamp_ns"]-plan_item["stamp_ns"])/1e9,
            "plan_start_xy_m": list(points[0]),
            "plan_end_xy_m": list(points[-1]),
            "plan_pose_count": len(points),
            "costmap_unknown_samples": int(unknown_samples),
            "costmap_lethal_samples": int(lethal_samples),
            "minimum_robot_clearance_m": clearance,
            "minimum_clearance_point_xy_m": path_clearance(static_metadata, path)[1],
            "gate_crossings": crossings}


def summarize_goal_plan_costmaps(plans, costmaps, expected_goal):
    summaries = []
    for plan in plans:
        path = plan["message"]
        points = [(float(pose.pose.position.x), float(pose.pose.position.y))
                  for pose in path.poses]
        if (len(points) < 2 or points[0][0] >= probe.NARROW_GATE_X
            or math.dist(points[-1], expected_goal) > 0.20):
            continue
        costmap = nearest_preceding(costmaps, plan["bag_time_ns"])
        if costmap is None:
            continue
        item = summarize_plan_costmap(plan, costmap, costmap["metadata"])
        item["costmap_occupancy_clear"] = (
            item["costmap_unknown_samples"] == 0
            and item["costmap_lethal_samples"] == 0)
        item["crosses_gate_once"] = len(item["gate_crossings"]) == 1
        item["gate_clearance_pass"] = (item["crosses_gate_once"]
                                         and item["gate_crossings"][0]["minimum_wall_clearance_m"] >= probe.MIN_ROBOT_CLEARANCE)
        item["passage_plan_candidate"] = (
            item["costmap_occupancy_clear"] and item["crosses_gate_once"]
            and item["gate_clearance_pass"]
            and item["minimum_robot_clearance_m"] >= probe.MIN_ROBOT_CLEARANCE)
        summaries.append(item)
    return summaries


def inspect_bag(bag_path, expected_start, expected_goal):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))
    message_types = {item.name: get_message(item.type)
                     for item in reader.get_all_topics_and_types()}
    required = {"/map", "/plan", "/global_costmap/costmap", "/scan"}
    missing = sorted(required-set(message_types))
    if missing:
        raise ValueError(f"rosbag is missing required topics: {missing}")

    plans, maps, costmaps = [], [], []
    while reader.has_next():
        topic, serialized, bag_time = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(serialized, message_types[topic])
        item = {"message": message, "stamp_ns": message_stamp_ns(message),
                "bag_time_ns": bag_time}
        if topic == "/plan":
            plans.append(item)
        elif topic == "/map":
            try:
                metadata, data = probe.map_values(message)
                metadata["data"] = array("b", data)
                maps.append({**item, "metadata": metadata})
            except (AttributeError, TypeError, ValueError):
                continue
        elif topic == "/global_costmap/costmap":
            try:
                metadata, data = probe.map_values(message)
                metadata["lethal_cells"] = sum(value == 100 for value in data)
                metadata["inflated_cost_cells"] = sum(50 <= value < 100 for value in data)
                metadata["occupied_cells"] = metadata["lethal_cells"]
                metadata["data"] = array("b", data)
                costmaps.append({**item, "metadata": metadata})
            except (AttributeError, TypeError, ValueError):
                continue

    candidates = []
    for item in plans:
        path = item["message"]
        points = [(float(pose.pose.position.x), float(pose.pose.position.y))
                  for pose in path.poses]
        if (len(points) >= 2
                and math.dist(points[0], expected_start) <= 0.20
                and math.dist(points[-1], expected_goal) <= 0.20):
            candidates.append(item)
    if not candidates:
        raise ValueError("bag contains no global plan matching the requested goal endpoints")
    selected = min(candidates, key=lambda item: item["bag_time_ns"])
    path = selected["message"]
    # Bag record time disambiguates the SLAM and fresh-Nav2 runs, whose /clock
    # and message header stamps restart when Fortress is relaunched.
    selected_map = nearest(maps, selected["bag_time_ns"])
    selected_costmap = nearest(costmaps, selected["bag_time_ns"])
    if selected_map is None or selected_costmap is None:
        raise ValueError("no recorded static map or global costmap near the selected plan")

    map_metadata, costmap_metadata = selected_map["metadata"], selected_costmap["metadata"]
    plan_costmap_evidence = []
    for plan in candidates:
        costmap = nearest(costmaps, plan["bag_time_ns"])
        if costmap is not None:
            plan_costmap_evidence.append(
                summarize_plan_costmap(plan, costmap, map_metadata))
    minimum_clearance, closest_point, static_unknown = path_clearance(map_metadata, path)
    sample_points = []
    seen = set()
    for item in static_unknown:
        point = (round(item["x_m"], 3), round(item["y_m"], 3))
        if point not in seen:
            seen.add(point)
            sample_points.append(point)
        if len(sample_points) >= 10:
            break
    pose_occupancy = []
    for pose in path.poses:
        x, y = float(pose.pose.position.x), float(pose.pose.position.y)
        pose_occupancy.append({
            "x_m": x, "y_m": y,
            "static_map": probe.occupancy_at(map_metadata, map_metadata["data"], x, y),
            "global_costmap": probe.occupancy_at(
                costmap_metadata, costmap_metadata["data"], x, y)})
    return {
        "status": "PASS_READ_ONLY_BAG_ANALYSIS",
        "bag": str(bag_path),
        "scope": "G4_SIMULATION_ONLY",
        "matching_plan_count": len(candidates),
        "all_goal_plan_costmap_summaries": summarize_goal_plan_costmaps(
            plans, costmaps, expected_goal),
        "matching_plan_costmap_evidence": plan_costmap_evidence,
        "selected_plan": {
            "stamp_ns": selected["stamp_ns"],
            "bag_time_ns": selected["bag_time_ns"],
            "frame_id": path.header.frame_id,
            "pose_count": len(path.poses),
            "start_xy_m": list(expected_start),
            "end_xy_m": list(expected_goal),
            "minimum_robot_clearance_m": minimum_clearance,
            "minimum_clearance_point_xy_m": closest_point,
            "static_unknown_samples": static_unknown,
            "pose_occupancy_samples": pose_occupancy,
        },
        "nearest_static_map": {
            "stamp_ns": selected_map["stamp_ns"],
            "record_delta_to_plan_seconds": (
                selected_map["bag_time_ns"]-selected["bag_time_ns"])/1e9,
            "header_stamp_delta_to_plan_seconds": (
                selected_map["stamp_ns"]-selected["stamp_ns"])/1e9,
            "metadata": {key: value for key, value in map_metadata.items() if key != "data"},
        },
        "nearest_global_costmap": {
            "stamp_ns": selected_costmap["stamp_ns"],
            "record_delta_to_plan_seconds": (
                selected_costmap["bag_time_ns"]-selected["bag_time_ns"])/1e9,
            "header_stamp_delta_to_plan_seconds": (
                selected_costmap["stamp_ns"]-selected["stamp_ns"])/1e9,
            "metadata": {key: value for key, value in costmap_metadata.items() if key != "data"},
        },
        "global_costmap_timeline_at_static_unknown_plan_samples": costmap_timeline(
            costmaps, selected, sample_points),
        "global_costmap_timeline_after_plan": costmap_timeline_after_plan(
            costmaps, selected, sample_points),
        "passage_cross_section_at_gate": passage_cross_section(
            map_metadata, costmap_metadata, probe.NARROW_GATE_X),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--start", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    report = inspect_bag(args.bag, tuple(args.start), tuple(args.goal))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(f"PASS [G4_BAG_ANALYSIS_READ_ONLY]: {args.report}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"FAIL [G4_BAG_ANALYSIS_READ_ONLY]: {type(exc).__name__}: {exc}", file=sys.stderr)
        sys.exit(1)
