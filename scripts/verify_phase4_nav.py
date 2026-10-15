#!/usr/bin/env python3
"""Bounded headless Nav2 readiness check for the Fortress simulation."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time


def run(args, report):
    import rclpy
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init(args=[])
    node = Node("linorobot2_g4_nav_readiness",
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    state_names = ("map_server", "amcl", "controller_server", "planner_server",
                   "behavior_server", "bt_navigator", "global_costmap/global_costmap",
                   "local_costmap/local_costmap", "velocity_smoother", "waypoint_follower")
    clients = {name: node.create_client(GetState, f"/{name}/get_state") for name in state_names}
    state_futures = {}
    samples = {"clock": 0, "clock_advances": 0, "scan": 0, "map": 0}
    clock_last = None
    cmd_nonzero = []
    maps = []

    def on_clock(message):
        nonlocal clock_last
        stamp = message.clock.sec*1_000_000_000+message.clock.nanosec
        if clock_last is not None and stamp > clock_last:
            samples["clock_advances"] += 1
        clock_last = stamp
        samples["clock"] += 1

    def on_scan(message):
        samples["scan"] += 1
        if len(message.ranges) < 180 or not message.header.frame_id:
            report["errors"].append("invalid LaserScan ray count/frame")

    def on_map(message):
        samples["map"] += 1
        maps.append({"width": message.info.width, "height": message.info.height,
                     "resolution": message.info.resolution, "frame_id": message.header.frame_id,
                     "known_cells": sum(1 for value in message.data if value >= 0),
                     "occupied_cells": sum(1 for value in message.data if value >= 65)})

    def on_command(message):
        if any(abs(value) > 1e-8 for value in
               (message.linear.x, message.linear.y, message.angular.z)):
            cmd_nonzero.append({"linear_x": message.linear.x, "angular_z": message.angular.z})

    node.create_subscription(Clock, "/clock", on_clock, 10)
    node.create_subscription(LaserScan, "/scan", on_scan, 10)
    node.create_subscription(OccupancyGrid, "/map", on_map, 10)
    node.create_subscription(Twist, "/cmd_vel", on_command, 10)
    deadline = time.monotonic()+args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for name, client in clients.items():
                if name not in state_futures and client.service_is_ready():
                    state_futures[name] = client.call_async(GetState.Request())
            for name, future in list(state_futures.items()):
                if future.done() and (future.result() is None
                                      or future.result().current_state.id != 3):
                    del state_futures[name]
            active = {name for name, future in state_futures.items()
                      if future.done() and future.result() is not None
                      and future.result().current_state.id == 3}
            if (len(active) == len(state_names) and samples["clock_advances"] >= 10
                    and samples["scan"] >= 10 and samples["map"] >= 1):
                break
        active = {name: future.result().current_state.label
                  for name, future in state_futures.items()
                  if future.done() and future.result() is not None}
        missing_active = sorted(name for name in state_names if active.get(name) != "active")
        if missing_active:
            raise TimeoutError(f"Nav2 lifecycle nodes not active before deadline: {missing_active}")
        if samples["clock_advances"] < 10 or samples["scan"] < 10 or samples["map"] < 1:
            raise TimeoutError(f"missing fresh /clock, /scan or /map data: {samples}")
        if report["errors"]:
            raise ValueError("sensor/scan validation failed: " + "; ".join(report["errors"]))
        latest_map = maps[-1]
        if (latest_map["frame_id"].lstrip("/") != "map" or latest_map["width"] == 0
                or latest_map["height"] == 0 or latest_map["resolution"] <= 0
                or latest_map["known_cells"] == 0):
            raise ValueError(f"invalid or empty static map: {latest_map}")
        if cmd_nonzero:
            raise ValueError(f"unexpected nonzero `/cmd_vel` before a goal: {cmd_nonzero[0]}")
        report.update({"pass": True, "lifecycle": active, "samples": samples,
                       "map": latest_map, "nonzero_commands_before_goal": cmd_nonzero})
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 5.0 <= args.timeout <= 180.0:
        parser.error("timeout must be in [5, 180] wall seconds")
    if os.environ.get("ROS_DOMAIN_ID") != "42" or not os.environ.get("IGN_PARTITION", "").startswith("linobot2_g4_"):
        parser.error("require ROS_DOMAIN_ID=42 and an isolated linobot2_g4_ partition")
    report = {"pass": False, "status": "RUNNING", "scope": "G4_SIMULATION_ONLY",
              "errors": [], "hardware_topics_or_devices": "none"}
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
