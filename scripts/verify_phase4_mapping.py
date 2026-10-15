#!/usr/bin/env python3
"""Bounded Fortress SLAM map-acquisition and safety evidence probe."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

from verify_phase3 import angle_delta, odom_progress, pose_yaw, projected_translation
from verify_sim_motion import MotionChecks, read_model_pose
from phase4_geometry import (
    NARROW_PASSAGE_WALL_RECTS,
    TEST_BOX_CENTER as OBSTACLE_CENTER,
    TEST_BOX_HALF_SIZE,
    WEST_ANCHOR_CENTER as G4_MAP_ANCHOR_CENTER,
    WEST_ANCHOR_HALF_SIZE as G4_MAP_ANCHOR_HALF_SIZE,
)

NODE_NAME = "linorobot2_g4_mapping_probe"
LINEAR_LIMIT = 0.08
ANGULAR_LIMIT = 0.30
TURN_RADIANS = 1.52
ROBOT_MARGIN = 0.30
MIN_NEW_KNOWN_CELLS = 200
MIN_OCCUPIED_MAP_CELLS = 20
MAP_FAR_SIDE_METERS = 5.0


def world_clearance(pose):
    px, py = pose["position"][:2]

    def rectangle_clearance(center, half_size):
        dx = max(abs(px-center[0])-half_size[0], 0.0)
        dy = max(abs(py-center[1])-half_size[1], 0.0)
        return math.hypot(dx, dy)-ROBOT_MARGIN

    return min([rectangle_clearance(OBSTACLE_CENTER, TEST_BOX_HALF_SIZE),
                rectangle_clearance(G4_MAP_ANCHOR_CENTER, G4_MAP_ANCHOR_HALF_SIZE)]
               + [rectangle_clearance(center, half_size)
                  for center, half_size in NARROW_PASSAGE_WALL_RECTS])


def map_summary(message):
    width, height = int(message.info.width), int(message.info.height)
    resolution = float(message.info.resolution)
    data = list(message.data)
    origin_pose = message.info.origin
    origin = (float(origin_pose.position.x), float(origin_pose.position.y))
    q = (float(origin_pose.orientation.x), float(origin_pose.orientation.y),
         float(origin_pose.orientation.z), float(origin_pose.orientation.w))
    norm = math.sqrt(sum(value*value for value in q))
    if (width <= 0 or height <= 0 or len(data) != width*height
            or not math.isfinite(resolution) or resolution <= 0
            or not all(math.isfinite(value) for value in origin)
            or not all(math.isfinite(value) for value in q) or abs(norm-1.0) > 0.01
            or abs(q[0]) > 1e-3 or abs(q[1]) > 1e-3
            or message.header.frame_id.lstrip("/") != "map"):
        raise ValueError("SLAM OccupancyGrid has invalid dimensions, resolution, origin or frame")
    known = [value for value in data if value >= 0]
    if any(value < -1 or value > 100 for value in data):
        raise ValueError("SLAM OccupancyGrid contains values outside [-1,100]")
    yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]),
                     1-2*(q[1]*q[1]+q[2]*q[2]))
    metadata = {"width": width, "height": height, "resolution": resolution,
                "origin": [*origin, yaw], "frame_id": message.header.frame_id,
                "known_cells": len(known),
                "occupied_cells": sum(value >= 65 for value in known)}
    return metadata, data


def find_box_cell(metadata, data):
    """Find occupied cells near the Fortress test box's visible left face."""
    resolution = metadata["resolution"]
    origin_x, origin_y, yaw = metadata["origin"]
    width, height = metadata["width"], metadata["height"]
    world_x = OBSTACLE_CENTER[0]-TEST_BOX_HALF_SIZE[0]
    world_y = OBSTACLE_CENTER[1]
    dx, dy = world_x-origin_x, world_y-origin_y
    x0 = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/resolution)
    y0 = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/resolution)
    x_radius = max(1, math.ceil(0.20/resolution))
    y_radius = max(1, math.ceil((TEST_BOX_HALF_SIZE[1]-0.05)/resolution))
    for y in range(max(0, y0-y_radius), min(height, y0+y_radius+1)):
        for x in range(max(0, x0-x_radius), min(width, x0+x_radius+1)):
            value = data[y*width+x]
            if value >= 65:
                return {"x_cell": x, "y_cell": y, "occupancy": value}
    return None


def validate_map_growth(initial_metadata, mapped_metadata):
    newly_known = mapped_metadata["known_cells"]-initial_metadata["known_cells"]
    if (newly_known < MIN_NEW_KNOWN_CELLS
            or mapped_metadata["occupied_cells"] < MIN_OCCUPIED_MAP_CELLS):
        raise RuntimeError("SLAM map has insufficient new coverage or occupied cells: "
                           f"newly_known={newly_known}, map={mapped_metadata}")
    return newly_known


def assert_publishers(node, expected, report):
    observed = {}
    for topic, names in expected.items():
        actual = sorted({item.node_name.lstrip("/")
                         for item in node.get_publishers_info_by_topic(topic)})
        if set(actual) != set(names):
            raise RuntimeError(f"unexpected ROS publisher ownership for {topic}: {actual}, expected {sorted(names)}")
        observed[topic] = actual
    report.setdefault("critical_topic_publishers", {}).update(observed)


def run(args, report):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, LaserScan
    from tf2_msgs.msg import TFMessage
    from verify_camera_gui import OwnershipGuard

    guard = OwnershipGuard(args.supervisor_pid, partition_prefix="linobot2_g4_")
    guard.check()
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(NODE_NAME)
    types = {"Twist": Twist, "Odometry": Odometry, "Clock": Clock,
             "JointState": JointState, "TFMessage": TFMessage}
    checks = MotionChecks(node, guard, report, lambda: None, types)
    map_samples = []
    map_updates = 0
    scan_times = []
    map_errors = []

    def on_map(message):
        nonlocal map_updates
        try:
            metadata, data = map_summary(message)
            map_samples.append({"time": time.monotonic(), "metadata": metadata, "data": data})
            map_updates += 1
            report["latest_map"] = metadata
            report["map_update_count"] = map_updates
            if len(map_samples) > 3:
                del map_samples[:-3]
        except (AttributeError, TypeError, ValueError) as exc:
            map_errors.append(str(exc))

    def on_scan(message):
        if len(message.ranges) < 180 or not message.header.frame_id:
            checks.error = checks.error or "invalid or frameless LaserScan"
        scan_times.append(time.monotonic())

    node.create_subscription(LaserScan, "/scan", on_scan, 10)
    node.create_subscription(OccupancyGrid, "/map", on_map, 10)
    try:
        checks.warmup(timeout=30.0)
        checks.assert_publishers_isolated(NODE_NAME)
        deadline = time.monotonic()+30
        while time.monotonic() < deadline and (not map_samples or len(scan_times) < 5):
            checks.spin(0.1)
        if map_errors:
            raise ValueError(map_errors[-1])
        if not map_samples:
            raise TimeoutError("slam_toolbox did not publish an initial /map")
        if len(scan_times) < 5 or time.monotonic()-scan_times[-1] > 2.0:
            raise TimeoutError("/scan was not fresh before mapping motion")
        assert_publishers(node, {
            "/clock": {"parameter_bridge"}, "/scan": {"parameter_bridge"},
            "/map": {"slam_toolbox"}, "/odom/unfiltered": {"parameter_bridge"},
            "/odom": {"ekf_filter_node"},
            "/tf": {"ekf_filter_node", "robot_state_publisher", "slam_toolbox"}}, report)
        initial_map = map_samples[-1]
        report["initial_map"] = initial_map["metadata"]
        initial_updates = map_updates

        segments = []
        start_world = read_model_pose(time.monotonic()+7)
        min_clearance = world_clearance(start_world)
        if min_clearance < 0:
            raise RuntimeError(f"initial robot pose too close to test obstacle: {min_clearance:.3f}m")
        route = (("cover_west_of_start", "straight", 0.55, -LINEAR_LIMIT, 0.0),
             ("approach_box", "straight", 0.80, LINEAR_LIMIT, 0.0),
                 ("turn_south", "turn", TURN_RADIANS, 0.0, -ANGULAR_LIMIT),
                 ("pass_south_of_box", "straight", 1.10, LINEAR_LIMIT, 0.0),
                 ("turn_east", "turn", TURN_RADIANS, 0.0, ANGULAR_LIMIT),
                 ("map_far_side", "straight", MAP_FAR_SIDE_METERS, LINEAR_LIMIT, 0.0),
                 ("turn_north", "turn", TURN_RADIANS, 0.0, ANGULAR_LIMIT),
                 ("return_to_goal_lane", "straight", 0.90, LINEAR_LIMIT, 0.0))
        for name, kind, target, linear, angular in route:
            guard.check()
            checks.assert_publishers_isolated(NODE_NAME)
            start_world = read_model_pose(time.monotonic()+7)
            start_odom = checks.odom_snapshot()["filtered"]
            if start_odom is None:
                raise RuntimeError("filtered odometry unavailable during SLAM mapping")
            trajectory_samples = [{"progress_m": 0.0, "pose": start_world,
                                   "obstacle_clearance_m": world_clearance(start_world)}]
            next_pose_sample = 0.25 if kind == "straight" else math.inf
            begin = time.monotonic()
            deadline = begin+(max(24.0, target*45.0) if kind == "straight" else 36.0)
            next_publish = begin
            next_guard = begin
            progress = 0.0
            travel_direction = 1.0 if linear >= 0 else -1.0
            turn_sign = 1.0 if angular > 0 else -1.0
            while progress < target:
                now = time.monotonic()
                if now >= deadline:
                    raise TimeoutError(f"mapping {name} did not reach {target:.3f}")
                if now >= next_guard:
                    guard.check()
                    next_guard = now+0.5
                if now >= next_publish:
                    checks.publish(linear, angular)
                    next_publish = now+0.1
                checks.spin(min(0.05, max(0.0, deadline-now)))
                current = checks.odom_samples["filtered"][-1]
                if kind == "straight":
                    progress = odom_progress("straight", start_odom, current, travel_direction)
                    if progress >= next_pose_sample:
                        guard.check()
                        sample_pose = read_model_pose(time.monotonic()+3.0)
                        sample_progress = travel_direction*projected_translation(
                            start_world, sample_pose)
                        dx = sample_pose["position"][0]-start_world["position"][0]
                        dy = sample_pose["position"][1]-start_world["position"][1]
                        yaw = pose_yaw(start_world)
                        cross_track = -dx*math.sin(yaw)+dy*math.cos(yaw)
                        clearance = world_clearance(sample_pose)
                        if (sample_progress < next_pose_sample-0.20
                                or sample_progress > target+0.35
                                or abs(cross_track) > 0.20 or clearance < 0):
                            raise RuntimeError(
                                f"mapping {name} ground-truth corridor sample out of bounds: "
                                f"progress={sample_progress:.3f}, cross_track={cross_track:.3f}, "
                                f"clearance={clearance:.3f}")
                        trajectory_samples.append({"progress_m": sample_progress,
                                                   "pose": sample_pose,
                                                   "obstacle_clearance_m": clearance})
                        next_pose_sample += 0.25
                else:
                    progress = turn_sign*angle_delta(start_odom["yaw"], current["yaw"])
                maximum = target+0.25 if kind == "straight" else 1.90
                if progress < -0.05 or progress > maximum:
                    raise RuntimeError(f"mapping {name} exceeded signed motion bound: {progress:.3f}")
            command_end = time.monotonic()
            checks.stop(0.4)
            checks.pump(0.2)
            end_world = read_model_pose(time.monotonic()+7)
            clearance = world_clearance(end_world)
            if kind == "straight":
                end_progress = travel_direction*projected_translation(start_world, end_world)
                trajectory_samples.append({"progress_m": end_progress,
                                           "pose": end_world,
                                           "obstacle_clearance_m": clearance})
                clearance = min(sample["obstacle_clearance_m"]
                                for sample in trajectory_samples)
            min_clearance = min(min_clearance, clearance)
            if clearance < 0:
                raise RuntimeError(f"robot approached obstacle during mapping: {clearance:.3f}m clearance")
            world_yaw_delta = angle_delta(pose_yaw(start_world), pose_yaw(end_world))
            if kind == "straight":
                world_progress = travel_direction*projected_translation(start_world, end_world)
                dx = end_world["position"][0]-start_world["position"][0]
                dy = end_world["position"][1]-start_world["position"][1]
                yaw = pose_yaw(start_world)
                cross_track = -dx*math.sin(yaw)+dy*math.cos(yaw)
                if (world_progress < target*0.5 or world_progress > target+0.35
                        or abs(cross_track) > 0.20 or abs(world_yaw_delta) > 0.25):
                    raise RuntimeError(f"mapping {name} ground-truth translation out of bounds: "
                                       f"progress={world_progress:.3f}, cross_track={cross_track:.3f}, "
                                       f"yaw={world_yaw_delta:.3f}")
            else:
                world_progress = turn_sign*world_yaw_delta
                world_translation = math.dist(start_world["position"][:2], end_world["position"][:2])
                if world_progress < target*0.5 or world_progress > 1.90 or world_translation > 0.18:
                    raise RuntimeError(f"mapping {name} ground-truth rotation out of bounds: "
                                       f"yaw={world_progress:.3f}, translation={world_translation:.3f}")
            segments.append({"name": name, "kind": kind, "target": target,
                             "odom_progress": progress, "wall_seconds": command_end-begin,
                             "ground_truth_start": start_world, "ground_truth_end": end_world,
                             "ground_truth_samples": trajectory_samples,
                             "ground_truth_progress": world_progress,
                             "min_obstacle_clearance_m": clearance, "pass": True})
            report["segments"] = list(segments)
            report["minimum_obstacle_clearance_m"] = min_clearance
            guard.check()

        checks.stop(0.5)
        checks.pump(0.5)
        settle_deadline = time.monotonic()+15
        route_end = time.monotonic()
        while time.monotonic() < settle_deadline and (
                not map_samples or map_samples[-1]["time"] < route_end
                or len(scan_times) < 5 or time.monotonic()-scan_times[-1] > 2.0):
            checks.spin(0.1)
        if map_errors:
            raise ValueError(map_errors[-1])
        if not map_samples:
            raise RuntimeError("SLAM map disappeared after mapping route")
        latest = map_samples[-1]
        metadata, data = latest["metadata"], latest["data"]
        obstacle_cell = find_box_cell(metadata, data)
        newly_known = metadata["known_cells"]-initial_map["metadata"]["known_cells"]
        report.update({"segments": segments, "map": metadata,
                       "newly_known_cells": newly_known,
                       "map_update_count": map_updates,
                       "test_box_occupied_cell_near_left_face": obstacle_cell,
                       "minimum_obstacle_clearance_m": min_clearance,
                       "clock_advances": checks.clock_advances,
                       "scan_samples": len(scan_times),
                       "safety_limits": {"linear_x_mps": LINEAR_LIMIT,
                                         "angular_z_rps": ANGULAR_LIMIT,
                                         "mapping_route": [segment["name"] for segment in segments],
                                         "turn_radians": TURN_RADIANS,
                                         "robot_radius_margin_m": ROBOT_MARGIN,
                                         "hardware_topics_or_devices": "none"}})
        validate_map_growth(initial_map["metadata"], metadata)
        if latest["time"] <= initial_map["time"] or map_updates <= initial_updates:
            raise RuntimeError("SLAM map did not update after the bounded mapping route")
        if (metadata == initial_map["metadata"] and data == initial_map["data"]):
            raise RuntimeError("SLAM map callback repeated the initial grid without mapping changes")
        if obstacle_cell is None:
            raise RuntimeError("SLAM map has no occupied cell aligned with the Fortress test box")
        if len(scan_times) < 10 or time.monotonic()-scan_times[-1] > 2.0:
            raise RuntimeError("LaserScan stream stale or insufficient at end of mapping")
        report.update({"pass": True, "status": "PASS"})
    finally:
        try:
            checks.stop(0.4)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--supervisor-pid", type=int, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    if (args.supervisor_pid <= 0 or os.environ.get("ROS_DOMAIN_ID") != "42"
            or not os.environ.get("IGN_PARTITION", "").startswith("linobot2_g4_")):
        parser.error("require a positive simulator supervisor, Domain 42 and isolated G4 partition")
    report = {"pass": False, "status": "RUNNING", "scope": "G4_SIMULATION_ONLY",
              "errors": [], "hardware_topics_or_devices": "none",
              "linear_speed_limit_mps": LINEAR_LIMIT,
              "angular_speed_limit_rps": ANGULAR_LIMIT}
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
