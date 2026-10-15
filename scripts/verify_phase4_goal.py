#!/usr/bin/env python3
"""Bounded Nav2 baseline/abort checks against a saved Fortress SLAM map."""

import argparse
from array import array
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from verify_sim_motion import POSE_TOPIC, parse_model_pose_stream, read_model_pose
from phase4_geometry import (
    MIN_ROBOT_CLEARANCE,
    NARROW_GATE_X,
    NARROW_GATE_HALF_THICKNESS,
    NARROW_PASSAGE_GOAL,
    NARROW_PASSAGE_HALF_WIDTH,
    NARROW_PASSAGE_HIGH_Y,
    NARROW_PASSAGE_LATERAL_BUDGET,
    NARROW_PASSAGE_LOW_Y,
    NARROW_PASSAGE_WALL_RECTS,
    NARROW_PASSAGE_WIDTH,
    ROBOT_RADIUS,
    TEST_BOX_CENTER as OBSTACLE_CENTER,
    TEST_BOX_HALF_SIZE,
)
OBSTACLE_HALF_SIZE = TEST_BOX_HALF_SIZE[0]

NODE_NAME = "linorobot2_g4_goal_probe"
LINEAR_LIMIT = 0.08
ANGULAR_LIMIT = 0.30
GOAL_X = 0.55
GOAL_Y = 0.0
AVOID_GOAL_CANDIDATES = tuple(
    (x, y) for x in (2.80, 2.75, 2.70, 2.85)
    for y in (-1.05, -1.00, -0.95, -0.90, -0.84, -0.79, -0.74, -0.70))
GOAL_ARRIVAL_TOLERANCE = 0.20
MAX_TF_AGE_SECONDS = 1.0
MAX_TF_FUTURE_SECONDS = 3.5
NAV2_FOOTPRINT_FRAMES = {"local_costmap": "odom", "global_costmap": "map"}
MAX_FOOTPRINT_RECEIVE_AGE_SECONDS = 2.0
UNREACHABLE_MAX_DISPLACEMENT = 0.25
UNREACHABLE_STOP_TOLERANCE = 0.02
EXPECTED_PREGOAL_CMD_PUBLISHERS = {
    "behavior_server", "command_timeout", "velocity_smoother"}


def map_values(message):
    width, height = int(message.info.width), int(message.info.height)
    resolution = float(message.info.resolution)
    origin = message.info.origin
    q = (float(origin.orientation.x), float(origin.orientation.y),
         float(origin.orientation.z), float(origin.orientation.w))
    norm = math.sqrt(sum(value*value for value in q))
    roll = math.atan2(2*(q[3]*q[0]+q[1]*q[2]), 1-2*(q[0]*q[0]+q[1]*q[1]))
    pitch_sin = 2*(q[3]*q[1]-q[2]*q[0])
    pitch = math.asin(max(-1.0, min(1.0, pitch_sin)))
    position = (float(origin.position.x), float(origin.position.y))
    if (width <= 0 or height <= 0 or len(message.data) != width*height
            or not math.isfinite(resolution) or resolution <= 0
            or message.header.frame_id.lstrip("/") != "map"
            or not all(math.isfinite(value) for value in q)
            or not all(math.isfinite(value) for value in position)
            or abs(norm-1.0) > 0.01 or abs(roll) > 1e-3 or abs(pitch) > 1e-3):
        raise ValueError("saved OccupancyGrid has invalid geometry, orientation or map frame")
    data = list(message.data)
    if any(value < -1 or value > 100 for value in data):
        raise ValueError("OccupancyGrid values must be in [-1,100]")
    yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))
    metadata = {"width": width, "height": height, "resolution": resolution,
                "origin": [position[0], position[1], yaw],
                "frame_id": message.header.frame_id,
                "known_cells": sum(value >= 0 for value in data),
                "occupied_cells": sum(value >= 65 for value in data)}
    return metadata, data


def occupancy_at(metadata, data, x, y):
    ox, oy, yaw = metadata["origin"]
    dx, dy = x-ox, y-oy
    local_x = math.cos(yaw)*dx + math.sin(yaw)*dy
    local_y = -math.sin(yaw)*dx + math.cos(yaw)*dy
    col = math.floor(local_x/metadata["resolution"])
    row = math.floor(local_y/metadata["resolution"])
    if not (0 <= col < metadata["width"] and 0 <= row < metadata["height"]):
        return None
    return data[row*metadata["width"]+col]


def nav2_costmap_cell_kind(value):
    """In an OccupancyGrid costmap, 50–99 are inflated costs, not occupied cells."""
    if value is None:
        return "out_of_map"
    if value == -1:
        return "unknown"
    if not 0 <= value <= 100:
        raise ValueError(f"Nav2 costmap value must be in [-1,100], got {value}")
    if value == 100:
        return "lethal"
    if value >= 50:
        return "inflated"
    return "free"


def validate_free_path(metadata, data, start=(0.0, 0.0), goal=(GOAL_X, GOAL_Y)):
    distance = math.dist(start, goal)
    count = max(1, math.ceil(distance/0.025))
    samples = []
    for index in range(count+1):
        ratio = index/count
        x = start[0]+(goal[0]-start[0])*ratio
        y = start[1]+(goal[1]-start[1])*ratio
        value = occupancy_at(metadata, data, x, y)
        if value is None or value < 0 or value >= 50:
            raise ValueError(f"planned line path is outside known free map at ({x:.3f},{y:.3f}): {value}")
        samples.append({"x": x, "y": y, "occupancy": value})
    return samples


def obstacle_clearance(x, y):
    def rectangle_clearance(center, half_size):
        dx = max(abs(x-center[0])-half_size[0], 0.0)
        dy = max(abs(y-center[1])-half_size[1], 0.0)
        return math.hypot(dx, dy)-ROBOT_RADIUS

    clearances = [rectangle_clearance(OBSTACLE_CENTER, TEST_BOX_HALF_SIZE)]
    clearances.extend(rectangle_clearance(center, half_size)
                      for center, half_size in NARROW_PASSAGE_WALL_RECTS)
    return min(clearances)


def validate_unreachable_target(metadata, data):
    """Use the physical box interior, not an arbitrary out-of-map goal."""
    x, y = OBSTACLE_CENTER
    value = occupancy_at(metadata, data, x, y)
    if value is None or (value != -1 and value < 65):
        raise ValueError(f"unreachable box interior must be in-map unknown/occupied: {value}")
    return {"x": x, "y": y, "occupancy": value,
            "reason": "physical Fortress test box interior; unknown is not free"}


def validate_unreachable_motion(start_pose, samples, limit=UNREACHABLE_MAX_DISPLACEMENT):
    start = start_pose["position"][:2]
    positions = [sample["pose"]["position"][:2] for sample in samples]
    if not all(math.isfinite(value) for point in [start]+positions for value in point):
        raise ValueError("nonfinite unreachable-goal ground truth")
    maximum = max((math.dist(start, point) for point in positions), default=0.0)
    clearance = min((obstacle_clearance(*point) for point in positions),
                    default=obstacle_clearance(*start))
    if maximum > limit or clearance < MIN_ROBOT_CLEARANCE:
        raise RuntimeError(f"unsafe unreachable recovery: displacement={maximum:.3f}m, "
                           f"clearance={clearance:.3f}m")
    return {"maximum_displacement_m": maximum, "minimum_robot_clearance_m": clearance,
            "displacement_limit_m": limit, "samples": len(samples)}


def validate_no_motion_commands(commands):
    nonzero = [item for item in commands if not item.get("zero")]
    if nonzero:
        raise RuntimeError("fail-closed unreachable policy emitted nonzero /cmd_vel")
    return {"command_samples": len(commands), "nonzero_command_samples": 0}


def validate_unreachable_result(accepted, status, zeros, stop_samples, commands=()):
    # Numeric action_msgs GoalStatus avoids a ROS dependency in contract tests.
    if not accepted or status != 6:
        raise RuntimeError(f"unreachable goal must be accepted then ABORTED: "
                           f"accepted={accepted}, status={status}")
    motion = validate_no_motion_commands(commands)
    if (len(stop_samples) < 3
            or any(not item.get("zero") or tuple(item.get("values") or ()) != (0, 0, 0)
                   for item in zeros)):
        raise RuntimeError("unreachable abort lacks measured stopped samples or valid zeros")
    if motion["nonzero_command_samples"] and not zeros:
        raise RuntimeError("unreachable abort after motion lacks final zero command")
    stopped = validate_unreachable_motion(
        stop_samples[0]["pose"], stop_samples, UNREACHABLE_STOP_TOLERANCE)
    stopped["explicit_zero_after_result"] = bool(zeros)
    stopped["stop_basis"] = ("final_zero_command" if zeros
                              else "no_nonzero_command_emitted")
    stopped["no_motion_command_samples"] = motion["command_samples"]
    return stopped


def validate_unreachable_planner_log(text):
    target = f"({OBSTACLE_CENTER[0]:.2f}, {OBSTACLE_CENTER[1]:.2f})"
    expected = f"Planning algorithm GridBased failed to generate a valid path to {target}"
    matches = [line for line in text.splitlines() if expected in line]
    forbidden = ("Transform data too old", "worldToMap failed", "Sensor origin",
                 "Controller patience", "Failed to obtain robot pose")
    if not matches or any(error in text for error in forbidden):
        raise RuntimeError("unreachable abort lacks target-specific planner failure or has TF/costmap fault")
    return {"failed_planner_attempts": len(matches), "last_failure": matches[-1]}


def has_known_safe_grid_path(metadata, data, start, goal):
    """Check 8-connected free-space reachability without corner cutting."""
    width, height = metadata["width"], metadata["height"]
    ox, oy, yaw = metadata["origin"]
    resolution = metadata["resolution"]

    def world_to_cell(point):
        dx, dy = point[0]-ox, point[1]-oy
        local_x = math.cos(yaw)*dx+math.sin(yaw)*dy
        local_y = -math.sin(yaw)*dx+math.cos(yaw)*dy
        return math.floor(local_x/resolution), math.floor(local_y/resolution)

    def safe(col, row):
        if not (0 <= col < width and 0 <= row < height):
            return False
        if min(col+0.5, row+0.5, width-col-0.5, height-row-0.5) \
                * resolution < ROBOT_RADIUS:
            return False
        occupancy = data[row*width+col]
        if occupancy < 0 or occupancy >= 50:
            return False
        local_x, local_y = (col+0.5)*resolution, (row+0.5)*resolution
        x = ox+math.cos(yaw)*local_x-math.sin(yaw)*local_y
        y = oy+math.sin(yaw)*local_x+math.cos(yaw)*local_y
        return obstacle_clearance(x, y) >= MIN_ROBOT_CLEARANCE

    source, destination = world_to_cell(start), world_to_cell(goal)
    if not safe(*source) or not safe(*destination):
        return False
    queue = deque([source])
    visited = {source}
    while queue:
        col, row = queue.popleft()
        if (col, row) == destination:
            return True
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbor = (col+dx, row+dy)
                if neighbor in visited or not safe(*neighbor):
                    continue
                if dx and dy and (not safe(col+dx, row) or not safe(col, row+dy)):
                    continue
                visited.add(neighbor)
                queue.append(neighbor)
    return False


def select_known_free_avoidance_goal(metadata, data, start=(GOAL_X, GOAL_Y)):
    """Choose the easternmost mapped-free and connected south-lane point."""
    candidates = []
    for x, y in AVOID_GOAL_CANDIDATES:
        occupancy = occupancy_at(metadata, data, x, y)
        clearance = obstacle_clearance(x, y)
        connected = (occupancy is not None and 0 <= occupancy < 50
                     and clearance >= MIN_ROBOT_CLEARANCE
                     and has_known_safe_grid_path(metadata, data, start, (x, y)))
        candidates.append({"x": x, "y": y, "occupancy": occupancy,
                           "clearance_m": clearance, "connected": connected})
        if connected:
            return x, y, occupancy
    raise ValueError("no known-free, safe obstacle-avoidance goal on the measured south lane: "
                     + json.dumps(candidates, separators=(",", ":"), allow_nan=False))


def validate_narrow_passage_goal(metadata, data, start):
    x, y = NARROW_PASSAGE_GOAL
    occupancy = occupancy_at(metadata, data, x, y)
    if occupancy is None or occupancy < 0 or occupancy >= 50:
        raise ValueError(f"narrow-passage goal must be known-free in sensor-derived map: {occupancy}")
    if obstacle_clearance(x, y) < MIN_ROBOT_CLEARANCE:
        raise ValueError("narrow-passage goal lacks required geometric clearance")
    if not has_known_safe_grid_path(metadata, data, start, (x, y)):
        raise ValueError("sensor-derived map has no known-safe path through narrow passage")
    return {"x": x, "y": y, "occupancy": occupancy,
            "passage_width_m": NARROW_PASSAGE_HIGH_Y-NARROW_PASSAGE_LOW_Y}


def iter_json_documents(lines):
    decoder = json.JSONDecoder()
    buffer = ""
    for line in lines:
        buffer += line
        offset = 0
        while True:
            while offset < len(buffer) and buffer[offset].isspace():
                offset += 1
            if offset == len(buffer):
                buffer = ""
                break
            try:
                document, end = decoder.raw_decode(buffer, offset)
            except json.JSONDecodeError:
                buffer = buffer[offset:]
                break
            yield document
            offset = end
    if buffer.strip():
        raise ValueError("truncated JSON document in Fortress pose stream")


def analyze_ground_truth_trajectory(samples):
    if len(samples) < 3:
        raise ValueError("too few Fortress ground-truth samples during obstacle navigation")
    minimum_clearance = math.inf
    minimum_clearance_point = None
    total_distance = 0.0
    maximum_sample_gap = 0.0
    for before, after in zip(samples, samples[1:]):
        start = before["pose"]["position"][:2]
        end = after["pose"]["position"][:2]
        distance = math.dist(start, end)
        total_distance += distance
        maximum_sample_gap = max(maximum_sample_gap, distance)
        steps = max(1, math.ceil(distance/0.025))
        for index in range(steps+1):
            ratio = index/steps
            x = start[0]+(end[0]-start[0])*ratio
            y = start[1]+(end[1]-start[1])*ratio
            clearance = obstacle_clearance(x, y)
            if clearance < minimum_clearance:
                minimum_clearance = clearance
                minimum_clearance_point = [x, y]
    if maximum_sample_gap > 0.25:
        raise ValueError(f"Fortress pose stream has an unobserved trajectory gap: {maximum_sample_gap:.3f}m")
    return {"samples": len(samples), "distance_m": total_distance,
            "minimum_robot_clearance_m": minimum_clearance,
            "minimum_clearance_point_xy_m": minimum_clearance_point,
            "maximum_sample_gap_m": maximum_sample_gap,
            "ground_truth_points_xy_m": [
                [float(value) for value in sample["pose"]["position"][:2]]
                for sample in samples]}


class GroundTruthStream:
    """Read only this partition's exact Fortress model pose from Ignition."""

    def __init__(self):
        self.samples = []
        self.errors = []
        self.process = subprocess.Popen(
            ["ign", "topic", "-e", "-t", POSE_TOPIC, "--json-output"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self.thread = threading.Thread(target=self._read, name="g4-pose-reader", daemon=True)
        self.thread.start()

    def _read(self):
        try:
            for document in iter_json_documents(self.process.stdout):
                pose = parse_model_pose_stream(json.dumps(document))
                sample_time = time.monotonic()
                if (not self.samples or sample_time-self.samples[-1]["time"] >= 0.15):
                    self.samples.append({"time": sample_time, "pose": pose})
        except Exception as exc:
            self.errors.append(f"{type(exc).__name__}: {exc}")

    def stop(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            raise TimeoutError("Fortress model-pose reader did not stop")
        if self.errors:
            raise RuntimeError("Fortress model-pose reader failed: " + "; ".join(self.errors))


def select_avoidance_plan(plans, expected_start, expected_goal):
    """Select the earliest global plan spanning from expected start to expected goal."""
    for plan in plans:
        points = [(float(p.pose.position.x), float(p.pose.position.y))
                  for p in plan.get("poses", [])]
        if (len(points) >= 2
                and math.dist(points[0], expected_start) <= 0.20
                and math.dist(points[-1], expected_goal) <= 0.20):
            return plan
    if plans:
        return plans[0]
    raise ValueError("no global planner paths published during navigation")


def analyze_avoidance_path(metadata, path_message, expected_start, expected_goal):
    if path_message["frame_id"].lstrip("/") != "map":
        raise ValueError("global planner path is not in map frame")
    points = []
    for pose in path_message["poses"]:
        if pose.header.frame_id and pose.header.frame_id.lstrip("/") != "map":
            raise ValueError("global planner path is not in map frame")
        points.append((float(pose.pose.position.x), float(pose.pose.position.y)))
    if len(points) < 2 or not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("global planner path is empty, short or nonfinite")
    start_dist = math.dist(points[0], expected_start)
    goal_dist = math.dist(points[-1], expected_goal)
    if start_dist > 0.20 or goal_dist > 0.20:
        raise ValueError(
            f"global planner path endpoints do not match the active goal: "
            f"start={points[0]} (expected {expected_start}, dist={start_dist:.3f}m > 0.20m); "
            f"end={points[-1]} (expected {expected_goal}, dist={goal_dist:.3f}m > 0.20m)")
    length = 0.0
    minimum_clearance = math.inf
    minimum_clearance_point = None
    for start, end in zip(points, points[1:]):
        segment = math.dist(start, end)
        length += segment
        steps = max(1, math.ceil(segment/0.025))
        for index in range(steps+1):
            ratio = index/steps
            x = start[0]+(end[0]-start[0])*ratio
            y = start[1]+(end[1]-start[1])*ratio
            occupancy = occupancy_at(metadata, path_message["data"], x, y)
            if occupancy is not None and occupancy >= 65:
                raise ValueError(f"global plan crosses occupied map cell at ({x:.3f},{y:.3f})")
            clearance = obstacle_clearance(x, y)
            if clearance < minimum_clearance:
                minimum_clearance = clearance
                minimum_clearance_point = [x, y]
    return {"poses": len(points), "length_m": length,
            "direct_distance_m": math.dist(points[0], points[-1]),
            "minimum_robot_clearance_m": minimum_clearance,
            "minimum_clearance_point_xy_m": minimum_clearance_point,
            "global_plan_points_xy_m": [list(point) for point in points],
            "start": list(points[0]), "end": list(points[-1])}


def passage_crossings(points):
    crossings = []
    for start, end in zip(points, points[1:]):
        if start[0] <= NARROW_GATE_X < end[0] or end[0] <= NARROW_GATE_X < start[0]:
            ratio = (NARROW_GATE_X-start[0])/(end[0]-start[0])
            y = start[1]+(end[1]-start[1])*ratio
            clearance = min(y-NARROW_PASSAGE_LOW_Y,
                            NARROW_PASSAGE_HIGH_Y-y)-ROBOT_RADIUS
            crossings.append({"x_m": NARROW_GATE_X, "y_m": y,
                              "minimum_wall_clearance_m": clearance})
    return crossings


def narrow_passage_path_evidence(metadata, path_message, global_costmap=None):
    points = [(float(item.pose.position.x), float(item.pose.position.y))
              for item in path_message["poses"]]
    source_data = path_message["data"]
    costmap_metadata = metadata
    costmap_data = source_data
    costmap_source = "saved_sensor_derived_map"
    costmap_age = None
    scan_age = None
    costmap_stamp_age = None
    costmap_errors = []
    costmap_publishers = []
    if global_costmap is not None:
        costmap_metadata = global_costmap["metadata"]
        costmap_data = global_costmap["data"]
        costmap_source = "live_nav2_global_costmap"
        costmap_age = global_costmap.get("receive_age_at_plan_s")
        scan_age = global_costmap.get("scan_age_at_plan_s")
        clock_ns = global_costmap.get("clock_ns_at_plan")
        stamp_ns = global_costmap.get("stamp_ns")
        costmap_publishers = list(global_costmap.get("publishers", []))
        if costmap_metadata.get("frame_id", "").lstrip("/") != "map":
            costmap_errors.append("global costmap frame is not map")
        if (costmap_age is None or not math.isfinite(costmap_age)
                or not 0.0 <= costmap_age <= MAX_FOOTPRINT_RECEIVE_AGE_SECONDS):
            costmap_errors.append("global costmap was stale/unavailable at plan time")
        if (scan_age is None or not math.isfinite(scan_age)
                or not 0.0 <= scan_age <= MAX_FOOTPRINT_RECEIVE_AGE_SECONDS):
            costmap_errors.append("LaserScan was stale/unavailable at plan time")
        if clock_ns is None or stamp_ns is None:
            costmap_errors.append("global costmap timestamp unavailable at plan time")
        else:
            costmap_stamp_age = (clock_ns-stamp_ns)/1_000_000_000
            if (costmap_stamp_age > MAX_TF_AGE_SECONDS
                    or costmap_stamp_age < -MAX_TF_FUTURE_SECONDS):
                costmap_errors.append(
                    f"global costmap timestamp stale at plan time: {costmap_stamp_age:.3f}s")
        if (len(costmap_publishers) != 1
                or costmap_publishers[0].rsplit("/", 1)[-1] != "global_costmap"):
            costmap_errors.append(
                f"unexpected global costmap publisher ownership: {costmap_publishers}")
    crossings = passage_crossings(points)
    unknown_samples = []
    occupied_samples = []
    free_sample_count = 0
    inflated_cost_sample_count = 0
    static_unknown_count = 0
    sensor_cleared_unknown_count = 0
    for start, end in zip(points, points[1:]):
        steps = max(1, math.ceil(math.dist(start, end)/0.025))
        for index in range(steps+1):
            ratio = index/steps
            x = start[0]+(end[0]-start[0])*ratio
            y = start[1]+(end[1]-start[1])*ratio
            static_value = occupancy_at(metadata, source_data, x, y)
            map_value = occupancy_at(costmap_metadata, costmap_data, x, y)
            if static_value == -1:
                static_unknown_count += 1
            elif static_value is None or static_value >= 50:
                occupied_samples.append({"x_m": x, "y_m": y,
                                         "occupancy": static_value,
                                         "source": "static_map"})
            costmap_kind = nav2_costmap_cell_kind(map_value)
            if costmap_kind in ("out_of_map", "unknown"):
                if len(unknown_samples) < 50:
                    unknown_samples.append({"x_m": x, "y_m": y,
                                            "occupancy": map_value,
                                            "static_map_occupancy": static_value})
            elif costmap_kind == "lethal":
                if len(occupied_samples) < 50:
                    occupied_samples.append({"x_m": x, "y_m": y,
                                             "occupancy": map_value,
                                             "static_map_occupancy": static_value,
                                             "source": "global_costmap_lethal"})
            else:
                free_sample_count += 1
                if costmap_kind == "inflated":
                    inflated_cost_sample_count += 1
                if static_value == -1:
                    sensor_cleared_unknown_count += 1
    return {"global_plan_pose_count": len(points),
            "global_plan_points_xy_m": [list(point) for point in points],
            "passage_crossings": crossings,
            "passage_width_m": NARROW_PASSAGE_HIGH_Y-NARROW_PASSAGE_LOW_Y,
            "clearance_at_gate_crossing_m": min(
                (item["minimum_wall_clearance_m"] for item in crossings), default=None),
            "static_map_unknown_sample_count": static_unknown_count,
            "sensor_cleared_unknown_sample_count": sensor_cleared_unknown_count,
            "known_free_plan_sample_count": free_sample_count,
            "inflated_cost_plan_sample_count": inflated_cost_sample_count,
            "unknown_plan_samples": unknown_samples,
            "occupied_or_out_of_map_samples": occupied_samples,
            "plan_costmap_source": costmap_source,
            "global_costmap_receive_age_at_plan_s": costmap_age,
            "global_costmap_stamp_age_at_plan_s": costmap_stamp_age,
            "laser_scan_receive_age_at_plan_s": scan_age,
            "global_costmap_publishers": costmap_publishers,
            "costmap_validation_errors": costmap_errors}


def analyze_narrow_passage_path(metadata, path_message, expected_start, expected_goal,
                                global_costmap=None):
    path = analyze_avoidance_path(metadata, path_message, expected_start, expected_goal)
    points = [(float(item.pose.position.x), float(item.pose.position.y))
              for item in path_message["poses"]]
    evidence = narrow_passage_path_evidence(metadata, path_message, global_costmap)
    if evidence["costmap_validation_errors"]:
        raise ValueError("narrow-passage planner-costmap evidence invalid: "
                         + "; ".join(evidence["costmap_validation_errors"]))
    if evidence["occupied_or_out_of_map_samples"]:
        item = evidence["occupied_or_out_of_map_samples"][0]
        raise ValueError("narrow-passage plan enters occupied/out-of-map cells at "
                         f"({item['x_m']:.3f},{item['y_m']:.3f}): {item['occupancy']}")
    if evidence["unknown_plan_samples"]:
        item = evidence["unknown_plan_samples"][0]
        raise ValueError("narrow-passage plan leaves known-free planner costmap at "
                         f"({item['x_m']:.3f},{item['y_m']:.3f}): {item['occupancy']}")
    crossings = evidence["passage_crossings"]
    if len(crossings) != 1:
        raise ValueError(f"global plan must cross the narrow-passage gate exactly once: {crossings}")
    if crossings[0]["minimum_wall_clearance_m"] < MIN_ROBOT_CLEARANCE:
        raise ValueError(f"global plan violates narrow-passage clearance: {crossings[0]}")
    path["passage_crossings"] = crossings
    path["passage_width_m"] = NARROW_PASSAGE_HIGH_Y-NARROW_PASSAGE_LOW_Y
    path["plan_costmap_source"] = evidence["plan_costmap_source"]
    path["static_map_unknown_sample_count"] = evidence["static_map_unknown_sample_count"]
    path["sensor_cleared_unknown_sample_count"] = (
        evidence["sensor_cleared_unknown_sample_count"])
    path["known_free_plan_sample_count"] = evidence["known_free_plan_sample_count"]
    path["inflated_cost_plan_sample_count"] = evidence["inflated_cost_plan_sample_count"]
    path["global_costmap_receive_age_at_plan_s"] = evidence["global_costmap_receive_age_at_plan_s"]
    path["laser_scan_receive_age_at_plan_s"] = evidence["laser_scan_receive_age_at_plan_s"]
    return path


def select_narrow_passage_plan(plans, metadata, expected_goal,
                               minimum_clearance=MIN_ROBOT_CLEARANCE,
                               max_pose_age=0.5):
    """Select a replan whose start, fresh sensor-cleared costmap, and clearance agree."""
    rejected = []
    expected_goal = tuple(expected_goal)
    for candidate in plans:
        points = [(float(item.pose.position.x), float(item.pose.position.y))
                  for item in candidate.get("poses", [])]
        if len(points) < 2 or math.dist(points[-1], expected_goal) > 0.20:
            continue
        gt = candidate.get("ground_truth_pose")
        now = candidate.get("time")
        pose_age = (abs(now-gt.get("time", -math.inf))
                if gt is not None and now is not None else math.inf)
        if (gt is None or now is None
            or pose_age > max_pose_age):
            rejected.append({"start_xy_m": list(points[0]),
                             "reason": "fresh ground-truth pose unavailable"})
            continue
        gt_xy = tuple(float(value) for value in gt["pose"]["position"][:2])
        if gt_xy[0] >= NARROW_GATE_X:
            rejected.append({"start_xy_m": list(points[0]), "ground_truth_xy_m": list(gt_xy),
                             "reason": "plan observed after the robot crossed the gate"})
            continue
        start_error = math.dist(points[0], gt_xy)
        if start_error > 0.20:
            rejected.append({"start_xy_m": list(points[0]),
                             "ground_truth_xy_m": list(gt_xy),
                             "start_to_ground_truth_m": start_error,
                             "reason": "plan start disagrees with independent pose"})
            continue
        try:
            summary = analyze_narrow_passage_path(
                metadata, candidate, points[0], expected_goal,
                global_costmap=candidate.get("global_costmap"))
        except (ValueError, RuntimeError) as exc:
            rejected.append({"start_xy_m": list(points[0]),
                             "ground_truth_xy_m": list(gt_xy),
                             "start_to_ground_truth_m": start_error,
                             "reason": f"{type(exc).__name__}: {exc}"})
            continue
        if summary["minimum_robot_clearance_m"] < minimum_clearance:
            rejected.append({"start_xy_m": list(points[0]),
                             "ground_truth_xy_m": list(gt_xy),
                             "start_to_ground_truth_m": start_error,
                             "minimum_robot_clearance_m": summary["minimum_robot_clearance_m"],
                             "minimum_clearance_point_xy_m": summary["minimum_clearance_point_xy_m"],
                             "reason": "plan clearance below unchanged G4 minimum"})
            continue
        summary["ground_truth_start_xy_m"] = list(gt_xy)
        summary["plan_start_to_ground_truth_m"] = start_error
        summary["ground_truth_sample_signed_offset_at_plan_s"] = now-gt["time"]
        return candidate, summary, rejected[-50:]
    raise ValueError("no fresh replanned passage path has known/free costmap and "
                     f"at least {minimum_clearance:.3f}m geometric clearance: "
                     + json.dumps(rejected[-50:], separators=(",", ":"), allow_nan=False))


def analyze_narrow_passage_trajectory(samples):
    points = [sample["pose"]["position"][:2] for sample in samples]
    crossings = passage_crossings(points)
    if len(crossings) != 1:
        raise ValueError(f"executed trajectory must cross the narrow-passage gate exactly once: {crossings}")
    if crossings[0]["minimum_wall_clearance_m"] < MIN_ROBOT_CLEARANCE:
        raise ValueError(f"executed trajectory violates narrow-passage clearance: {crossings[0]}")
    return {"passage_crossings": crossings,
            "passage_width_m": NARROW_PASSAGE_HIGH_Y-NARROW_PASSAGE_LOW_Y,
            "minimum_robot_clearance_m": crossings[0]["minimum_wall_clearance_m"],
            "ground_truth_trajectory_samples": len(samples)}


def checked_command(message):
    values = (float(message.linear.x), float(message.linear.y), float(message.angular.z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite /cmd_vel command")
    if abs(values[0]) > LINEAR_LIMIT+0.005 or abs(values[1]) > 0.005 \
            or abs(values[2]) > ANGULAR_LIMIT+0.005:
        raise ValueError(f"Nav2 command exceeds G4 profile limits: {values}")
    return values


def validate_tf_freshness(clock_ns, tf_stamps_ns):
    """Require both dynamic localization transforms to track simulation time."""
    required = {"map->odom", "odom->base_footprint"}
    missing = required-set(tf_stamps_ns)
    if clock_ns is None or missing:
        raise RuntimeError("localization TF freshness unavailable: "
                           f"clock_ns={clock_ns}, missing={sorted(missing)}")
    ages = {}
    for edge in sorted(required):
        age = (clock_ns-tf_stamps_ns[edge])/1_000_000_000
        if age > MAX_TF_AGE_SECONDS or age < -MAX_TF_FUTURE_SECONDS:
            raise RuntimeError(f"stale localization TF {edge}: age={age:.3f}s; "
                               f"allowed=[{-MAX_TF_FUTURE_SECONDS:.1f},"
                               f"{MAX_TF_AGE_SECONDS:.1f}]s")
        ages[edge] = age
    return ages


def localization_tf_snapshot(clock_ns, tf_stamps_ns, tf_received=None, now=None,
                             tf_update_counts=None, tf_updates_at_start=None,
                             tf_stamps_at_start=None):
    """Capture sim-time TF age and wall-time receive age, including missing edges."""
    now = time.monotonic() if now is None else now
    result = {"clock_ns": clock_ns, "transforms": {}}
    for edge in ("map->odom", "odom->base_footprint"):
        stamp = tf_stamps_ns.get(edge)
        received = (tf_received or {}).get(edge)
        updates = (tf_update_counts or {}).get(edge, 0)
        updates_at_start = (tf_updates_at_start or {}).get(edge)
        stamp_at_start = (tf_stamps_at_start or {}).get(edge)
        result["transforms"][edge] = {
            "stamp_ns": stamp,
            "age_seconds": ((clock_ns-stamp)/1_000_000_000
                            if clock_ns is not None and stamp is not None else None),
            "receive_age_seconds": (max(0.0, now-received)
                                    if received is not None else None),
            "updates_during_goal": (updates-updates_at_start
                                    if updates_at_start is not None else None),
            "stamp_delta_ns_during_goal": (stamp-stamp_at_start
                                           if stamp is not None and stamp_at_start is not None
                                           else None),
        }
    return result


def amcl_pose_snapshot(clock_ns, stamp_ns, received, count, count_at_start=None,
                       stamp_at_start=None, now=None):
    """Capture AMCL pose delivery and timestamp progress for one goal attempt."""
    now = time.monotonic() if now is None else now
    return {
        "stamp_ns": stamp_ns,
        "age_seconds": ((clock_ns-stamp_ns)/1_000_000_000
                        if clock_ns is not None and stamp_ns is not None else None),
        "receive_age_seconds": (max(0.0, now-received) if received is not None else None),
        "samples_during_goal": count-count_at_start if count_at_start is not None else None,
        "stamp_delta_ns_during_goal": (stamp_ns-stamp_at_start
                                       if stamp_ns is not None and stamp_at_start is not None
                                       else None),
    }


def footprint_sample(message, expected_frame, received=None):
    """Nav2 stamps published footprints with its internally resolved robot pose."""
    stamp = message.header.stamp
    points = message.polygon.points
    if (message.header.frame_id.lstrip("/") != expected_frame
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000
            or len(points) < 3
            or not all(math.isfinite(value) for point in points
                       for value in (point.x, point.y, point.z))):
        raise ValueError("invalid Nav2 footprint frame, timestamp or polygon")
    return {"frame_id": expected_frame,
            "stamp_ns": stamp.sec*1_000_000_000+stamp.nanosec,
            "received": time.monotonic() if received is None else received}


def nav2_footprint_snapshot(clock_ns, footprints, now=None):
    now = time.monotonic() if now is None else now
    return {name: {
        "frame_id": footprints.get(name, {}).get("frame_id"),
        "stamp_ns": footprints.get(name, {}).get("stamp_ns"),
        "age_seconds": ((clock_ns-footprints[name]["stamp_ns"])/1_000_000_000
                        if clock_ns is not None and name in footprints else None),
        "receive_age_seconds": (max(0.0, now-footprints[name]["received"])
                                if name in footprints else None),
    } for name in NAV2_FOOTPRINT_FRAMES}


def validate_nav2_footprints(clock_ns, footprints, now=None):
    """Fresh /tf alone does not prove Nav2's separate TF buffers are current."""
    snapshot = nav2_footprint_snapshot(clock_ns, footprints, now)
    for name, expected_frame in NAV2_FOOTPRINT_FRAMES.items():
        sample = snapshot[name]
        age = sample["age_seconds"]
        receive_age = sample["receive_age_seconds"]
        if age is None or receive_age is None:
            raise RuntimeError(f"Nav2 footprint freshness unavailable: {name}")
        if (sample["frame_id"] != expected_frame
                or age > MAX_TF_AGE_SECONDS or age < -MAX_TF_FUTURE_SECONDS
                or receive_age > MAX_FOOTPRINT_RECEIVE_AGE_SECONDS):
            raise RuntimeError(f"stale Nav2 footprint {name}: "
                               f"pose_age={age:.3f}s, receive_age={receive_age:.3f}s")
    return snapshot


def validate_goal_arrival(distance, zero_command_count):
    if not math.isfinite(distance) or distance < 0:
        raise ValueError("ground-truth goal distance must be finite and nonnegative")
    if distance > GOAL_ARRIVAL_TOLERANCE or zero_command_count < 1:
        raise RuntimeError("goal success lacks ground-truth arrival/final zero: "
                           f"distance={distance:.3f}m (limit={GOAL_ARRIVAL_TOLERANCE:.2f}m), "
                           f"zero_count={zero_command_count}")


def validate_successful_goal_stop(commands, post_result_samples, goal_finished,
                                  tolerance=UNREACHABLE_STOP_TOLERANCE):
    """Require a final zero after motion and independently measured stillness after success."""
    if not math.isfinite(goal_finished) or not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("goal stop observation time and tolerance must be finite and valid")
    zeros = [item for item in commands if item.get("zero")]
    nonzeros = [item for item in commands if not item.get("zero")]
    if not zeros:
        raise RuntimeError("successful goal lacks an observed final zero command")
    last_nonzero_time = max((item.get("time", -math.inf) for item in nonzeros),
                            default=-math.inf)
    final_zero = max(zeros, key=lambda item: item.get("time", -math.inf))
    if final_zero.get("time", -math.inf) <= last_nonzero_time:
        raise RuntimeError("successful goal lacks a zero command after its last nonzero command")
    post_result_nonzeros = [item for item in nonzeros
                            if item.get("time", -math.inf) >= goal_finished]
    stopped_samples = [sample for sample in post_result_samples
                       if sample.get("time", -math.inf) >= goal_finished]
    if len(stopped_samples) < 3:
        raise RuntimeError("successful goal lacks three post-result stillness samples")
    positions = [sample["pose"]["position"][:2] for sample in stopped_samples]
    if not all(math.isfinite(value) for point in positions for value in point):
        raise ValueError("successful goal has nonfinite post-result ground truth")
    maximum_drift = max((math.dist(positions[0], point) for point in positions[1:]),
                        default=0.0)
    if maximum_drift > tolerance:
        raise RuntimeError("successful goal did not remain stationary after result: "
                           f"drift={maximum_drift:.3f}m (limit={tolerance:.3f}m)")
    zero_after_motion = [item for item in zeros
                         if item.get("time", -math.inf) > last_nonzero_time]
    zero_after_result = [item for item in zero_after_motion
                         if item.get("time", -math.inf) >= goal_finished]
    return {"stop_basis": "zero_after_last_nonzero_plus_post_result_stillness",
            "last_nonzero_command_time": (last_nonzero_time if nonzeros else None),
            "final_zero_command": final_zero,
            "zero_command_count_after_last_nonzero": len(zero_after_motion),
            "zero_command_count_after_result": len(zero_after_result),
            "nonzero_command_count_after_result": len(post_result_nonzeros),
            "post_result_stillness_samples": len(stopped_samples),
            "post_result_max_drift_m": maximum_drift,
            "post_result_drift_tolerance_m": tolerance}


def goal_arrival_diagnostics(goal_label, target_xy, ground_truth_pose, distance,
                             zero_commands, trajectory_sample_count,
                             tf_transforms=None, amcl_pose=None):
    end_xy = [float(value) for value in ground_truth_pose["position"][:2]]
    target_xy = [float(value) for value in target_xy]
    diag = {
        "goal": goal_label,
        "nav2_action_status": "SUCCEEDED",
        "target_map_xy_m": target_xy,
        "fortress_ground_truth_end_xy_m": end_xy,
        "ground_truth_xy_error_vector_m": [end_xy[0]-target_xy[0], end_xy[1]-target_xy[1]],
        "ground_truth_distance_to_goal_m": float(distance),
        "arrival_tolerance_m": GOAL_ARRIVAL_TOLERANCE,
        "zero_command_count_after_result": len(zero_commands),
        "final_zero_command": zero_commands[-1] if zero_commands else None,
        "ground_truth_trajectory_samples": trajectory_sample_count,
    }
    if tf_transforms:
        diag["tf_transforms"] = dict(tf_transforms)
    if amcl_pose:
        diag["amcl_pose"] = list(amcl_pose)
    return diag


def summarize_goal_timeout(goal_label, goal, started, start_pose, pose_samples,
                           commands, plans, clock_advances, odom_samples):
    positions = [sample["pose"]["position"][:2] for sample in pose_samples]
    path_distance = sum(math.dist(a, b) for a, b in zip(positions, positions[1:]))
    end_pose = pose_samples[-1]["pose"] if pose_samples else start_pose
    start_xy = start_pose["position"][:2] if start_pose else None
    end_xy = end_pose["position"][:2] if end_pose else None
    displacement = math.dist(start_xy, end_xy) if start_xy and end_xy else None
    last_plan_length = None
    if plans:
        path = plans[-1]["poses"]
        points = [(item.pose.position.x, item.pose.position.y) for item in path]
        last_plan_length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return {
        "goal": goal_label,
        "target_map_xy_m": [goal[0], goal[1]],
        "wall_seconds": round(time.monotonic()-started, 3),
        "ground_truth_samples": len(pose_samples),
        "ground_truth_start_xy_m": start_xy,
        "ground_truth_end_xy_m": end_xy,
        "ground_truth_displacement_m": displacement,
        "ground_truth_path_distance_m": path_distance,
        "odom_samples": odom_samples,
        "sim_clock_advances": clock_advances,
        "cmd_vel_samples": len(commands),
        "nonzero_cmd_vel_samples": sum(not item["zero"] for item in commands),
        "max_abs_linear_x_mps": max((abs(item["values"][0]) for item in commands), default=0.0),
        "max_abs_angular_z_rps": max((abs(item["values"][2]) for item in commands), default=0.0),
        "planner_path_samples": len(plans),
        "last_planner_path_length_m": last_plan_length,
    }


def assert_publishers(node, expected, report):
    observed = {}
    for topic, names in expected.items():
        actual = sorted({item.node_name.lstrip("/")
                         for item in node.get_publishers_info_by_topic(topic)})
        if set(actual) != set(names):
            raise RuntimeError(f"unexpected ROS publisher ownership for {topic}: {actual}, expected {sorted(names)}")
        observed[topic] = actual
    report.setdefault("critical_topic_publishers", {}).update(observed)


def validate_pregoal_cmd_publishers(publisher_infos):
    """Require only the known Nav2 and watchdog publishers before a goal."""
    names = {item.node_name.strip().lstrip("/") for item in publisher_infos}
    if names != EXPECTED_PREGOAL_CMD_PUBLISHERS:
        raise RuntimeError(f"unexpected /cmd_vel publishers before goal: {sorted(names)}")
    return sorted(names)


def validate_global_costmap_publishers(names):
    observed = sorted({str(name).strip().lstrip("/") for name in names})
    owners = [name for name in observed if name.rsplit("/", 1)[-1] == "global_costmap"]
    if len(observed) != 1 or len(owners) != 1:
        raise RuntimeError(f"unexpected global costmap publisher ownership: {observed}")
    return observed


def run(args, report):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from verify_camera_gui import OwnershipGuard

    guard = OwnershipGuard(args.supervisor_pid, partition_prefix="linobot2_g4_")
    guard.check()
    rclpy.init(args=[])
    node = Node(NODE_NAME, parameter_overrides=[Parameter("use_sim_time", value=True)])
    map_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {"map": None, "map_error": None, "scan_count": 0, "last_scan": None,
             "global_costmap": None, "global_costmap_error": None,
             "global_costmap_publishers": [],
             "clock_ns": None, "clock_advances": 0, "clock_rewind": False,
             "clock_progress_at": time.monotonic(),
             "odom_count": 0, "tf": {}, "tf_stamps_ns": {},
             "tf_update_counts": {},
             "tf_transforms": {}, "amcl_pose": None,
             "amcl_pose_count": 0, "amcl_pose_stamp_ns": None,
             "amcl_pose_received": None,
             "commands": [], "plans": [],
             "footprints": {}, "footprint_error": None,
             "command_error": None}
    goal_handle = None
    result_future = None

    def on_map(message):
        try:
            state["map"] = map_values(message)
        except (AttributeError, TypeError, ValueError) as exc:
            state["map_error"] = str(exc)

    def on_global_costmap(message):
        try:
            metadata, data = map_values(message)
            metadata["lethal_cells"] = sum(value == 100 for value in data)
            metadata["inflated_cost_cells"] = sum(50 <= value < 100 for value in data)
            metadata["occupied_cells"] = metadata["lethal_cells"]
            state["global_costmap"] = {
                "metadata": metadata,
                "data": array("b", data),
                "stamp_ns": (message.header.stamp.sec*1_000_000_000
                             + message.header.stamp.nanosec),
                "received": time.monotonic(),
            }
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            state["global_costmap_error"] = str(exc)

    def on_scan(message):
        if len(message.ranges) < 180 or not message.header.frame_id:
            state["command_error"] = "invalid or frameless LaserScan"
        state["scan_count"] += 1
        state["last_scan"] = time.monotonic()

    def on_clock(message):
        stamp = message.clock.sec*1_000_000_000+message.clock.nanosec
        previous = state["clock_ns"]
        if previous is not None:
            if stamp < previous:
                state["clock_rewind"] = True
            elif stamp > previous:
                state["clock_advances"] += 1
                state["clock_progress_at"] = time.monotonic()
        state["clock_ns"] = stamp

    def on_odom(message):
        if (message.header.frame_id.lstrip("/") != "odom"
                or message.child_frame_id.lstrip("/") != "base_footprint"):
            state["command_error"] = "filtered odometry frame contract mismatch"
        state["odom_count"] += 1

    def on_tf(message):
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if (parent, child) in (("map", "odom"), ("odom", "base_footprint")):
                state["tf"][f"{parent}->{child}"] = time.monotonic()
                edge = f"{parent}->{child}"
                state["tf_update_counts"][edge] = state["tf_update_counts"].get(edge, 0)+1
                state["tf_stamps_ns"][edge] = (
                    transform.header.stamp.sec*1_000_000_000
                    + transform.header.stamp.nanosec)
                t = transform.transform.translation
                state["tf_transforms"][edge] = [
                    float(t.x), float(t.y), float(t.z)]

    def on_amcl_pose(message):
        p = message.pose.pose.position
        state["amcl_pose"] = [float(p.x), float(p.y), float(p.z)]
        state["amcl_pose_count"] += 1
        state["amcl_pose_stamp_ns"] = (
            message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec)
        state["amcl_pose_received"] = time.monotonic()

    def on_command(message):
        try:
            values = checked_command(message)
            state["commands"].append({"time": time.monotonic(), "values": values,
                                      "zero": values == (0.0, 0.0, 0.0)})
        except (TypeError, ValueError) as exc:
            state["command_error"] = str(exc)

    def on_plan(message):
        received = time.monotonic()
        costmap = state["global_costmap"]
        ground_truth_pose = (
            ground_truth_stream.samples[-1]
            if ground_truth_stream is not None and ground_truth_stream.samples
            else None)
        plan_costmap = None
        if costmap is not None:
            plan_costmap = dict(costmap)
            plan_costmap["receive_age_at_plan_s"] = received-costmap["received"]
            plan_costmap["scan_age_at_plan_s"] = (
                received-state["last_scan"] if state["last_scan"] is not None else None)
            plan_costmap["clock_ns_at_plan"] = state["clock_ns"]
            plan_costmap["publishers"] = list(state["global_costmap_publishers"])
        state["plans"].append({"time": received, "frame_id": message.header.frame_id,
                               "poses": list(message.poses),
                               "data": state["map"][1] if state["map"] else None,
                               "global_costmap": plan_costmap,
                               "ground_truth_pose": ground_truth_pose})

    def on_footprint(name, message):
        try:
            state["footprints"][name] = footprint_sample(
                message, NAV2_FOOTPRINT_FRAMES[name])
        except (AttributeError, TypeError, ValueError) as exc:
            state["footprint_error"] = f"{name}: {exc}"

    for name in NAV2_FOOTPRINT_FRAMES:
        node.create_subscription(
            PolygonStamped, f"/{name}/published_footprint",
            lambda message, name=name: on_footprint(name, message), 10)
    node.create_subscription(OccupancyGrid, "/map", on_map, map_qos)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap", on_global_costmap, 10)
    node.create_subscription(LaserScan, "/scan", on_scan, 10)
    node.create_subscription(Clock, "/clock", on_clock, 10)
    node.create_subscription(Odometry, "/odom", on_odom, 10)
    node.create_subscription(TFMessage, "/tf", on_tf, 100)
    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl_pose, 10)
    node.create_subscription(Twist, "/cmd_vel", on_command, 20)
    node.create_subscription(NavPath, "/plan", on_plan, 10)
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    ground_truth_stream = None

    def execute_goal(x, y, label, expect_abort=False):
        nonlocal goal_handle, result_future
        plan_start = time.monotonic()
        pose_sample_start = len(ground_truth_stream.samples)
        start_pose = (ground_truth_stream.samples[-1]["pose"]
                      if ground_truth_stream.samples else None)
        command_sample_start = len(state["commands"])
        clock_advances_start = state["clock_advances"]
        scan_samples_start = state["scan_count"]
        odom_samples_start = state["odom_count"]
        tf_updates_at_start = dict(state["tf_update_counts"])
        tf_stamps_at_start = dict(state["tf_stamps_ns"])
        amcl_pose_count_at_start = state["amcl_pose_count"]
        amcl_pose_stamp_at_start = state["amcl_pose_stamp_ns"]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        if expect_abort:
            if args.unreachable_behavior_tree is None:
                raise ValueError("unreachable goal requires the fail-closed behavior tree")
            goal.behavior_tree = str(args.unreachable_behavior_tree)
        send_future = client.send_goal_async(goal)
        accepted_deadline = time.monotonic()+20
        next_guard = time.monotonic()
        while not send_future.done() and time.monotonic() < accepted_deadline:
            if time.monotonic() >= next_guard:
                guard.check()
                next_guard = time.monotonic()+0.5
            rclpy.spin_once(node, timeout_sec=0.05)
        if not send_future.done():
            raise TimeoutError(f"Nav2 did not accept {label} within 20s")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Nav2 rejected the {label}")
        result_future = goal_handle.get_result_async()
        goal_deadline = time.monotonic()+args.goal_timeout
        next_guard = time.monotonic()

        def failure_diagnostics(action_status, failure_time=None):
            failure_time = time.monotonic() if failure_time is None else failure_time
            commands = state["commands"][command_sample_start:]
            plans = [item for item in state["plans"] if item["time"] >= plan_start]
            pose_samples = [sample for sample in
                            ground_truth_stream.samples[pose_sample_start:]
                            if sample["time"] >= plan_start]
            try:
                tf_publishers = sorted({item.node_name.strip().lstrip("/")
                                         for item in node.get_publishers_info_by_topic("/tf")})
            except Exception as exc:
                tf_publishers = [f"unavailable: {type(exc).__name__}: {exc}"]
            return {
                "goal": label,
                "target_map_xy_m": [x, y],
                "nav2_action_status": action_status,
                "sim_clock_ns": state["clock_ns"],
                "sim_clock_advances_during_goal": (
                    state["clock_advances"]-clock_advances_start),
                "sim_clock_wall_age_seconds": max(
                    0.0, failure_time-state["clock_progress_at"]),
                "localization_tf": localization_tf_snapshot(
                    state["clock_ns"], state["tf_stamps_ns"], state["tf"],
                    failure_time, state["tf_update_counts"],
                    tf_updates_at_start, tf_stamps_at_start),
                "nav2_footprints": nav2_footprint_snapshot(
                    state["clock_ns"], state["footprints"], failure_time),
                "nav2_footprint_error": state["footprint_error"],
                "tf_topic_publishers": tf_publishers,
                "amcl_pose": amcl_pose_snapshot(
                    state["clock_ns"], state["amcl_pose_stamp_ns"],
                    state["amcl_pose_received"], state["amcl_pose_count"],
                    amcl_pose_count_at_start, amcl_pose_stamp_at_start, failure_time),
                "scan_samples_during_goal": state["scan_count"]-scan_samples_start,
                "last_scan_wall_age_seconds": (
                    max(0.0, failure_time-state["last_scan"])
                    if state["last_scan"] is not None else None),
                "odom_samples_during_goal": state["odom_count"]-odom_samples_start,
                "ground_truth_samples_during_goal": len(pose_samples),
                "cmd_vel_samples_during_goal": len(commands),
                "nonzero_cmd_vel_samples_during_goal": sum(
                    not item["zero"] for item in commands),
                "planner_path_samples_during_goal": len(plans),
            }

        while not result_future.done():
            now = time.monotonic()
            if now >= goal_deadline:
                pose_samples = [sample for sample in
                                ground_truth_stream.samples[pose_sample_start:]
                                if sample["time"] >= plan_start]
                command_samples = state["commands"][command_sample_start:]
                plans = [item for item in state["plans"] if item["time"] >= plan_start]
                details = summarize_goal_timeout(
                    label, (x, y), plan_start, start_pose, pose_samples,
                    command_samples, plans,
                    state["clock_advances"]-clock_advances_start,
                    state["odom_count"]-odom_samples_start)
                details.update(failure_diagnostics("ACTIVE", now))
                report.setdefault("goal_attempts", []).append(details)
                raise TimeoutError(
                    f"Nav2 {label} exceeded the wall-clock deadline; "
                    f"progress={json.dumps(details, separators=(',', ':'), allow_nan=False)}")
            if now >= next_guard:
                guard.check()
                next_guard = now+0.5
            rclpy.spin_once(node, timeout_sec=min(0.05, goal_deadline-now))
            try:
                validate_tf_freshness(state["clock_ns"], state["tf_stamps_ns"])
                if state["footprint_error"]:
                    raise RuntimeError(state["footprint_error"])
                validate_nav2_footprints(state["clock_ns"], state["footprints"])
                if expect_abort:
                    if (not ground_truth_stream.samples or
                            time.monotonic()-ground_truth_stream.samples[-1]["time"] > 2.0):
                        raise RuntimeError("unreachable-goal ground truth stream is stale")
                    validate_unreachable_motion(
                        start_pose, ground_truth_stream.samples[pose_sample_start:])
            except RuntimeError as exc:
                diagnostics = failure_diagnostics("ACTIVE")
                report.setdefault("goal_attempts", []).append(diagnostics)
                raise RuntimeError(
                    f"{exc}; diagnostics="
                    f"{json.dumps(diagnostics, separators=(',', ':'), allow_nan=False)}") from exc
            if state["command_error"]:
                raise ValueError(state["command_error"])
            if expect_abort:
                validate_no_motion_commands(state["commands"][command_sample_start:])
            if state["clock_rewind"]:
                raise ValueError("simulation clock rewound during navigation")
            if expect_abort and (state["last_scan"] is None
                                 or time.monotonic()-state["last_scan"] > 2.0):
                raise RuntimeError("LaserScan stale during unreachable action")
            if time.monotonic()-state["clock_progress_at"] > 3.0:
                raise TimeoutError("simulation clock stopped during navigation")
            if ground_truth_stream.errors:
                raise RuntimeError("Fortress pose stream failed: "
                                   + "; ".join(ground_truth_stream.errors))
        result = result_future.result()
        expected_status = (GoalStatus.STATUS_ABORTED if expect_abort
                   else GoalStatus.STATUS_SUCCEEDED)
        if result is None or result.status != expected_status:
            status = None if result is None else result.status
            failure_time = time.monotonic()
            diagnostics = failure_diagnostics(status, failure_time)
            report.setdefault("goal_attempts", []).append(diagnostics)
            raise RuntimeError(
                f"Nav2 {label} unexpected terminal status={status}, expected={expected_status}; "
                f"diagnostics={json.dumps(diagnostics, separators=(',', ':'), allow_nan=False)}")
        try:
            tf_ages = validate_tf_freshness(state["clock_ns"], state["tf_stamps_ns"])
            if state["footprint_error"]:
                raise RuntimeError(state["footprint_error"])
            footprint_ages = validate_nav2_footprints(
                state["clock_ns"], state["footprints"])
        except RuntimeError as exc:
            diagnostics = failure_diagnostics(
                "ABORTED_BUT_STALE_TF" if expect_abort else "SUCCEEDED_BUT_STALE_TF")
            report.setdefault("goal_attempts", []).append(diagnostics)
            raise RuntimeError(
                f"Nav2 reported {label} terminal result with stale localization TF: {exc}; "
                f"diagnostics={json.dumps(diagnostics, separators=(',', ':'), allow_nan=False)}") from exc
        goal_finished = time.monotonic()
        action_commands = state["commands"][command_sample_start:]
        no_motion_policy = (validate_no_motion_commands(action_commands)
                    if expect_abort else None)
        zero_required = (expect_abort and no_motion_policy["nonzero_command_samples"] > 0)
        stop_deadline = goal_finished+3.0
        while time.monotonic() < stop_deadline and zero_required and not any(
                sample["zero"] and sample["time"] >= goal_finished for sample in state["commands"]):
            rclpy.spin_once(node, timeout_sec=min(0.05, stop_deadline-time.monotonic()))
            if expect_abort:
                guard.check()
                validate_unreachable_motion(start_pose, ground_truth_stream.samples[pose_sample_start:])
        if zero_required and not any(
                sample["zero"] and sample["time"] >= goal_finished for sample in state["commands"]):
            report.setdefault("goal_attempts", []).append(failure_diagnostics(int(result.status)))
            raise RuntimeError("unreachable abort lacks final zero within 3s")
        stop_observation_start = time.monotonic()
        observe_until = stop_observation_start+1.0
        while time.monotonic() < observe_until:
            guard.check()
            rclpy.spin_once(node, timeout_sec=0.05)
            validate_tf_freshness(state["clock_ns"], state["tf_stamps_ns"])
            validate_nav2_footprints(state["clock_ns"], state["footprints"])
            if state["command_error"] or state["footprint_error"]:
                raise RuntimeError(state["command_error"] or state["footprint_error"])
            if state["clock_rewind"] or time.monotonic()-state["clock_progress_at"] > 3.0:
                raise RuntimeError("simulation clock failed after terminal goal result")
        end_pose = read_model_pose(time.monotonic()+7)
        trajectory = [sample for sample in ground_truth_stream.samples[pose_sample_start:]
                  if sample["time"] >= plan_start]
        trajectory.append({"time": time.monotonic(), "pose": end_pose})
        distance = math.dist(end_pose["position"][:2], (x, y))
        zeros = [sample for sample in state["commands"]
                 if sample["zero"] and sample["time"] >= goal_finished]
        if expect_abort:
            diagnostics = failure_diagnostics(int(result.status))
            diagnostics.update({"action_accepted": bool(goal_handle.accepted),
                                "expected_outcome": "ABORTED",
                                "ground_truth_end": end_pose,
                                "zero_command_count_after_result": len(zeros),
                                "final_zero_command": zeros[-1] if zeros else None})
            report.setdefault("goal_attempts", []).append(diagnostics)
            motion = validate_unreachable_motion(start_pose, trajectory)
            stop_samples = [sample for sample in trajectory
                            if sample["time"] >= stop_observation_start]
            stopped = validate_unreachable_result(
                goal_handle.accepted, int(result.status), zeros, stop_samples,
                action_commands)
            analyze_ground_truth_trajectory(trajectory)  # Reject unobserved gaps.
            if any(not item["zero"] for item in state["commands"]
                   if item["time"] >= stop_observation_start):
                raise RuntimeError("nonzero command resumed after unreachable abort stop")
            if state["last_scan"] is None or time.monotonic()-state["last_scan"] > 2.0:
                raise RuntimeError("LaserScan stale after unreachable abort")
            if state["clock_advances"] <= clock_advances_start:
                raise RuntimeError("clock did not progress across abort and stop observation")
            planner_failure = validate_unreachable_planner_log(args.nav2_log.read_text())
            diagnostics.update({"motion": motion, "stopped_motion": stopped,
                                "no_motion_policy": no_motion_policy,
                                "planner_failure": planner_failure})
            goal_handle = None
            result_future = None
            return {"status": "ABORTED", "diagnostics": diagnostics}
        action_commands = state["commands"][command_sample_start:]
        post_result_samples = [sample for sample in trajectory
                               if sample["time"] >= goal_finished]
        arrival = goal_arrival_diagnostics(
            label, (x, y), end_pose, distance, zeros, len(trajectory),
            state["tf_transforms"], state["amcl_pose"])
        arrival["cmd_vel_samples_during_goal"] = len(action_commands)
        arrival["cmd_vel_command_tail"] = [
            {"time_from_goal_start_s": item["time"]-plan_start,
             "time_from_result_s": item["time"]-goal_finished,
             "values": item["values"], "zero": item["zero"]}
            for item in action_commands[-20:]]
        arrival["post_result_ground_truth_samples"] = len(post_result_samples)
        arrival["post_result_ground_truth_tail"] = [
            {"time_from_result_s": sample["time"]-goal_finished,
             "position_xy_m": sample["pose"]["position"][:2]}
            for sample in post_result_samples[-10:]]
        arrival["cmd_vel_nonzero_samples_received_after_result"] = sum(
            not item["zero"] and item["time"] >= goal_finished
            for item in action_commands)
        arrival["cmd_vel_zero_samples_received_after_result"] = sum(
            item["zero"] and item["time"] >= goal_finished
            for item in action_commands)
        arrival["tf_age_seconds_at_result"] = tf_ages
        arrival["nav2_footprints_at_result"] = footprint_ages
        report.setdefault("goal_attempts", []).append(arrival)
        stop_evidence = validate_successful_goal_stop(
            action_commands, post_result_samples, goal_finished)
        arrival["final_zero_command"] = stop_evidence["final_zero_command"]
        arrival["zero_command_count_after_last_nonzero"] = (
            stop_evidence["zero_command_count_after_last_nonzero"])
        arrival["post_result_stillness"] = stop_evidence
        try:
            validate_goal_arrival(
                distance, stop_evidence["zero_command_count_after_last_nonzero"])
        except RuntimeError as exc:
            raise RuntimeError(
                f"{label}: {exc}; target_map_xy_m={arrival['target_map_xy_m']}; "
                f"Fortress_ground_truth_end_xy_m={arrival['fortress_ground_truth_end_xy_m']}; "
                f"ground_truth_xy_error_vector_m={arrival['ground_truth_xy_error_vector_m']}; "
                f"trajectory_samples={len(trajectory)}") from exc
        fresh_plans = [plan for plan in state["plans"] if plan["time"] >= plan_start]
        goal_handle = None
        result_future = None
        return {"status": "SUCCEEDED", "ground_truth_end": end_pose,
                "distance_to_goal_m": distance, "zero_command": zeros[-1],
            "stop_proof": stop_evidence,
            "plans": fresh_plans, "ground_truth_trajectory": trajectory}

    try:
        ready_deadline = time.monotonic()+args.readiness_timeout
        while time.monotonic() < ready_deadline:
            guard.check()
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["command_error"]:
                raise ValueError(state["command_error"])
            if state["footprint_error"]:
                raise ValueError(state["footprint_error"])
            if state["global_costmap_error"]:
                raise ValueError("invalid Nav2 global costmap: "
                                 + state["global_costmap_error"])
            if (state["map"] and state["clock_advances"] >= 10
                    and state["scan_count"] >= 10 and state["odom_count"] >= 3
                    and state["global_costmap"] is not None
                    and set(state["tf"]) == {"map->odom", "odom->base_footprint"}
                    and client.server_is_ready()):
                try:
                    validate_tf_freshness(state["clock_ns"], state["tf_stamps_ns"])
                    validate_nav2_footprints(state["clock_ns"], state["footprints"])
                except RuntimeError:
                    continue
                break
        else:
            report["nav2_footprints"] = nav2_footprint_snapshot(
                state["clock_ns"], state["footprints"])
            raise TimeoutError("Nav2 action, map, clock, scan, odometry or localization TF not ready")
        if state["clock_rewind"]:
            raise ValueError("simulation clock rewound before goal")
        if time.monotonic()-state["clock_progress_at"] > 3.0:
            raise TimeoutError("simulation clock stopped before goal")
        if state["last_scan"] is None or time.monotonic()-state["last_scan"] > 2.0:
            raise TimeoutError("LaserScan is stale before goal")
        metadata, data = state["map"]
        path_cells = validate_free_path(metadata, data)
        unreachable_target = (validate_unreachable_target(metadata, data)
                      if args.scenario == "unreachable" else None)
        if state["commands"] and any(not command["zero"] for command in state["commands"]):
            raise ValueError("unsolicited nonzero /cmd_vel observed before goal")
        publisher_names = validate_pregoal_cmd_publishers(
            node.get_publishers_info_by_topic("/cmd_vel"))
        report["cmd_vel_publishers"] = publisher_names
        global_costmap_publishers = sorted({
            item.node_name.strip().lstrip("/")
            for item in node.get_publishers_info_by_topic("/global_costmap/costmap")})
        report["global_costmap_publishers_observed"] = list(global_costmap_publishers)
        report["global_costmap_publishers"] = validate_global_costmap_publishers(
            global_costmap_publishers)
        state["global_costmap_publishers"] = list(global_costmap_publishers)
        assert_publishers(node, {
            "/clock": {"parameter_bridge"}, "/scan": {"parameter_bridge"},
            "/map": {"map_server"}, "/odom": {"ekf_filter_node"},
            "/tf": {"ekf_filter_node", "robot_state_publisher", "amcl"}}, report)
        report["preflight"] = {"map": metadata, "free_path_samples": len(path_cells),
                               "scan_samples": state["scan_count"],
                               "odom_samples": state["odom_count"],
                               "global_costmap": {
                                   "map": state["global_costmap"]["metadata"],
                                   "stamp_ns": state["global_costmap"]["stamp_ns"],
                                   "receive_age_seconds": (
                                       time.monotonic()-state["global_costmap"]["received"]),
                                   "free_cells": sum(value >= 0 and value < 50
                                                      for value in state["global_costmap"]["data"]),
                                   "inflated_cost_cells": sum(50 <= value < 100
                                                               for value in state["global_costmap"]["data"]),
                                   "unknown_cells": sum(value < 0
                                                         for value in state["global_costmap"]["data"]),
                                   "lethal_cells": sum(value == 100
                                                         for value in state["global_costmap"]["data"])},
                               "nav2_footprints": validate_nav2_footprints(
                                   state["clock_ns"], state["footprints"]),
                               "tf_edges": sorted(state["tf"]),
                               "clock_advances": state["clock_advances"]}
        start_pose = read_model_pose(time.monotonic()+7)
        if math.dist(start_pose["position"][:2], (0.0, 0.0)) > 0.12:
            raise RuntimeError(f"simulator robot does not start at expected map origin: {start_pose}")
        ground_truth_stream = GroundTruthStream()
        stream_deadline = time.monotonic()+5.0
        while len(ground_truth_stream.samples) < 3 and time.monotonic() < stream_deadline:
            guard.check()
            rclpy.spin_once(node, timeout_sec=0.05)
        if ground_truth_stream.errors or len(ground_truth_stream.samples) < 3:
            raise TimeoutError("Fortress ground-truth pose stream was not ready before Nav2 goals")

        if unreachable_target is not None:
            report["unreachable_target"] = unreachable_target
            aborted = execute_goal(
                unreachable_target["x"], unreachable_target["y"],
                "physical-box unreachable goal", expect_abort=True)
            report.update({"pass": True, "unreachable_goal": aborted,
                           "map": metadata, "clock_advances": state["clock_advances"],
                           "scan_samples": state["scan_count"],
                           "safety_limits": {"linear_x_mps": LINEAR_LIMIT,
                                             "angular_z_rps": ANGULAR_LIMIT,
                                             "recovery_displacement_m": UNREACHABLE_MAX_DISPLACEMENT,
                                             "hardware_topics_or_devices": "none"}})
            return

        first_goal = execute_goal(GOAL_X, GOAL_Y, "known-free reachable goal")
        displacement = math.dist(start_pose["position"][:2],
                                 first_goal["ground_truth_end"]["position"][:2])
        if displacement < 0.25:
            raise RuntimeError(f"Nav2 success lacks measured goal displacement/arrival: "
                               f"displacement={displacement:.3f}m")
        second_start = first_goal["ground_truth_end"]
        avoid_goal_x, avoid_goal_y, second_goal_occupancy = (
            select_known_free_avoidance_goal(
                metadata, data, tuple(second_start["position"][:2])))
        state["plans"].clear()
        avoidance_goal = execute_goal(
            avoid_goal_x, avoid_goal_y,
            f"obstacle-avoiding reachable goal ({avoid_goal_x:.2f}, {avoid_goal_y:.2f})")
        if not avoidance_goal["plans"]:
            raise RuntimeError("global planner published no path for obstacle-avoidance goal")
        assert_publishers(node, {"/plan": {"planner_server"}}, report)
        selected_plan = select_avoidance_plan(
            avoidance_goal["plans"],
            tuple(second_start["position"][:2]), (avoid_goal_x, avoid_goal_y))
        obstacle_costmap = selected_plan.get("global_costmap")
        report["obstacle_avoidance_plan_points_xy_m"] = [
            [float(item.pose.position.x), float(item.pose.position.y)]
            for item in selected_plan.get("poses", [])]
        report["obstacle_avoidance_plan_costmap"] = ({
            "metadata": obstacle_costmap["metadata"],
            "stamp_ns": obstacle_costmap["stamp_ns"],
            "clock_ns_at_plan": obstacle_costmap["clock_ns_at_plan"],
            "receive_age_at_plan_s": obstacle_costmap["receive_age_at_plan_s"],
            "scan_age_at_plan_s": obstacle_costmap["scan_age_at_plan_s"],
            "publishers": obstacle_costmap["publishers"],
        } if obstacle_costmap is not None else None)
        report["obstacle_avoidance_ground_truth_points_xy_m"] = [
            [float(value) for value in sample["pose"]["position"][:2]]
            for sample in avoidance_goal["ground_truth_trajectory"]]
        plan = analyze_avoidance_path(
            metadata, selected_plan,
            tuple(second_start["position"][:2]), (avoid_goal_x, avoid_goal_y))
        executed_trajectory = analyze_ground_truth_trajectory(
            avoidance_goal["ground_truth_trajectory"])
        report["obstacle_avoidance_plan_evidence"] = plan
        report["obstacle_avoidance_trajectory_evidence"] = executed_trajectory
        report["obstacle_avoidance_goal_arrival"] = {
            "target_xy_m": [avoid_goal_x, avoid_goal_y],
            "ground_truth_end_xy_m": list(avoidance_goal["ground_truth_end"]["position"][:2]),
            "distance_to_goal_m": avoidance_goal["distance_to_goal_m"],
            "stop_proof": avoidance_goal["stop_proof"],
        }
        direct_distance = math.dist(second_start["position"][:2],
                                    (avoid_goal_x, avoid_goal_y))
        if plan["minimum_robot_clearance_m"] < 0.15:
            raise RuntimeError(f"global plan approaches Fortress obstacle unsafely: {plan}")
        if executed_trajectory["minimum_robot_clearance_m"] < 0.15:
            raise RuntimeError("executed Fortress trajectory approaches obstacle unsafely: "
                               f"{executed_trajectory}")
        if plan["length_m"] < direct_distance*1.02:
            raise RuntimeError(f"global plan did not measurably detour around obstacle: {plan}")
        narrow_preflight = validate_narrow_passage_goal(
            metadata, data, tuple(avoidance_goal["ground_truth_end"]["position"][:2]))
        state["plans"].clear()
        narrow_goal = execute_goal(
            narrow_preflight["x"], narrow_preflight["y"],
            f"narrow-passage goal ({narrow_preflight['x']:.2f}, {narrow_preflight['y']:.2f})")
        if not narrow_goal["plans"]:
            raise RuntimeError("global planner published no path for narrow-passage goal")
        assert_publishers(node, {"/plan": {"planner_server"}}, report)
        narrow_plan, narrow_plan_summary, rejected_narrow_plans = (
            select_narrow_passage_plan(
                narrow_goal["plans"], metadata,
                (narrow_preflight["x"], narrow_preflight["y"])))
        report["narrow_passage_plan_rejections"] = rejected_narrow_plans
        selected_global_costmap = narrow_plan.get("global_costmap")
        report["narrow_passage_plan_evidence"] = narrow_passage_path_evidence(
            metadata, narrow_plan, selected_global_costmap)
        report["narrow_passage_plan_costmap"] = ({
            "metadata": selected_global_costmap["metadata"],
            "stamp_ns": selected_global_costmap["stamp_ns"],
            "clock_ns_at_plan": selected_global_costmap["clock_ns_at_plan"],
            "receive_age_at_plan_s": selected_global_costmap["receive_age_at_plan_s"],
            "scan_age_at_plan_s": selected_global_costmap["scan_age_at_plan_s"],
            "publishers": selected_global_costmap["publishers"],
        } if selected_global_costmap is not None else None)
        report["narrow_passage_ground_truth_xy_m"] = [
            [float(value) for value in sample["pose"]["position"][:2]]
            for sample in narrow_goal["ground_truth_trajectory"]]
        report["narrow_passage_selected_plan_summary"] = narrow_plan_summary
        narrow_trajectory = analyze_ground_truth_trajectory(
            narrow_goal["ground_truth_trajectory"])
        narrow_trajectory["passage"] = analyze_narrow_passage_trajectory(
            narrow_goal["ground_truth_trajectory"])
        report["narrow_passage_trajectory_evidence"] = narrow_trajectory
        if narrow_trajectory["minimum_robot_clearance_m"] < MIN_ROBOT_CLEARANCE:
            raise RuntimeError(
                f"executed narrow-passage trajectory clearance is below minimum: {narrow_trajectory}")
        if state["last_scan"] is None or time.monotonic()-state["last_scan"] > 2.0:
            raise TimeoutError("LaserScan became stale after navigation")
        if state["clock_rewind"] or state["clock_advances"] < 10:
            raise RuntimeError("simulation clock failed to advance monotonically")
        report.update({"pass": True,
                       "reachable_goal": {"frame": "map", "x": GOAL_X, "y": GOAL_Y,
                                          "result": first_goal["status"],
                                          "ground_truth_end": first_goal["ground_truth_end"],
                                          "distance_to_goal_m": first_goal["distance_to_goal_m"],
                                          "zero_command": first_goal["zero_command"]},
                       "obstacle_avoidance_goal": {"frame": "map", "x": avoid_goal_x,
                                                   "y": avoid_goal_y, "result": avoidance_goal["status"],
                                                   "ground_truth_end": avoidance_goal["ground_truth_end"],
                                                   "distance_to_goal_m": avoidance_goal["distance_to_goal_m"],
                                                   "zero_command": avoidance_goal["zero_command"],
                                                   "global_plan": plan,
                                                   "executed_trajectory": executed_trajectory,
                                                   "goal_occupancy": second_goal_occupancy},
                       "narrow_passage_goal": {
                           "frame": "map", "x": narrow_preflight["x"],
                           "y": narrow_preflight["y"], "result": narrow_goal["status"],
                           "ground_truth_end": narrow_goal["ground_truth_end"],
                           "distance_to_goal_m": narrow_goal["distance_to_goal_m"],
                           "zero_command": narrow_goal["zero_command"],
                           "goal_occupancy": narrow_preflight["occupancy"],
                           "global_plan": narrow_plan_summary,
                           "executed_trajectory": narrow_trajectory},
                       "ground_truth_start": start_pose,
                       "ground_truth_end": narrow_goal["ground_truth_end"],
                       "ground_truth_displacement_m": math.dist(
                           start_pose["position"][:2],
                           narrow_goal["ground_truth_end"]["position"][:2]),
                       "zero_command_after_goal": narrow_goal["zero_command"],
                       "command_samples": len(state["commands"]),
                       "maximum_observed_abs_linear_x_mps": max(
                           (abs(item["values"][0]) for item in state["commands"]), default=0.0),
                       "maximum_observed_abs_angular_z_rps": max(
                           (abs(item["values"][2]) for item in state["commands"]), default=0.0),
                       "clock_advances": state["clock_advances"],
                       "scan_samples": state["scan_count"],
                       "map": metadata,
                       "safety_limits": {"linear_x_mps": LINEAR_LIMIT,
                                         "angular_z_rps": ANGULAR_LIMIT,
                                         "hardware_topics_or_devices": "none"}})
    finally:
        if goal_handle is not None and goal_handle.accepted and result_future is not None \
                and not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            cancel_deadline = time.monotonic()+5.0
            while not cancel_future.done() and time.monotonic() < cancel_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not cancel_future.done():
                raise TimeoutError("Nav2 goal cancellation response did not arrive")
            response = cancel_future.result()
            if response is None:
                raise RuntimeError("Nav2 returned an empty goal cancellation response")
            goal_id = bytes(goal_handle.goal_id.uuid)
            cancellation_accepted = any(
                bytes(item.goal_id.uuid) == goal_id for item in response.goals_canceling)
            if not cancellation_accepted and result_future.done():
                terminal = result_future.result()
                if terminal is None or terminal.status not in (
                        GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED,
                        GoalStatus.STATUS_ABORTED):
                    raise RuntimeError("Nav2 goal cancellation was rejected and goal is nonterminal")
                report["goal_cleanup"] = {"cancel_accepted": False,
                                          "terminal_status": int(terminal.status)}
            else:
                if not cancellation_accepted:
                    raise RuntimeError("Nav2 rejected cancellation while goal remained active")
                cancel_deadline = time.monotonic()+5.0
                while not result_future.done() and time.monotonic() < cancel_deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if not result_future.done():
                    raise TimeoutError("Nav2 goal did not reach a terminal state after cancellation")
                terminal = result_future.result()
                if terminal is None or terminal.status not in (
                        GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED,
                        GoalStatus.STATUS_ABORTED):
                    raise RuntimeError("Nav2 cancellation left the goal in a nonterminal state")
                report["goal_cleanup"] = {"cancel_accepted": True,
                                          "terminal_status": int(terminal.status)}
        try:
            if ground_truth_stream is not None:
                ground_truth_stream.stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--supervisor-pid", type=int, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--readiness-timeout", type=float, default=45.0)
    parser.add_argument("--goal-timeout", type=float, default=180.0)
    parser.add_argument("--scenario", choices=("baseline", "unreachable"), default="baseline")
    parser.add_argument("--nav2-log", type=Path,
                        help="required target-specific planner evidence for unreachable scenario")
    parser.add_argument("--unreachable-behavior-tree", type=Path,
                        help="goal-specific no-motion tree for the unreachable scenario")
    args = parser.parse_args(argv)
    if args.scenario == "unreachable":
        if args.nav2_log is None or not args.nav2_log.is_file():
            parser.error("unreachable scenario requires a readable --nav2-log")
        if (args.unreachable_behavior_tree is None
                or not args.unreachable_behavior_tree.is_file()):
            parser.error("unreachable scenario requires --unreachable-behavior-tree")
    if (args.supervisor_pid <= 0 or not math.isfinite(args.readiness_timeout)
            or not 10 <= args.readiness_timeout <= 90
            or not math.isfinite(args.goal_timeout) or not 15 <= args.goal_timeout <= 240):
        parser.error("invalid supervisor PID or wall-clock timeout bound")
    if (os.environ.get("ROS_DOMAIN_ID") != "42"
            or not os.environ.get("IGN_PARTITION", "").startswith("linobot2_g4_")):
        parser.error("require ROS_DOMAIN_ID=42 and an isolated linobot2_g4_ partition")
    report = {"pass": False, "status": "RUNNING", "scope": "G4_SIMULATION_ONLY",
              "scenario": args.scenario, "errors": [], "hardware_topics_or_devices": "none"}
    started = time.monotonic()
    previous = {}

    def interrupt(signum, _frame):
        raise KeyboardInterrupt(f"received signal {signum}")

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        run(args, report)
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        report["pass"] = False
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        report["elapsed_seconds"] = time.monotonic()-started
        report["status"] = "PASS" if report["pass"] else "FAIL"
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(f"{report['status']} [G4_SIMULATION_ONLY]: "
          + (str(args.report) if report["pass"] else "; ".join(report["errors"])), flush=True)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
