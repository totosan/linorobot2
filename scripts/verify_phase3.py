#!/usr/bin/env python3
"""Bounded Phase 3 SIL checks for square odometry, TF ownership and command recovery."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from verify_sim_motion import (JOINTS, MODEL, WORLD, MotionChecks, angle_delta,
                               projected_translation, read_model_pose, write_report,
                               yaw_from_quaternion)

NODE_NAME = "linorobot2_g3_safety_probe"
LINEAR_LIMIT = 0.08
ANGULAR_LIMIT = 0.30
SIDE_METERS = 0.12
TURN_RADIANS = 1.52
DRIVE_FRAME_X = -0.07


def pose_yaw(pose):
    return yaw_from_quaternion(pose["orientation"])


def odom_progress(kind, start, current, direction=1):
    if kind == "straight":
        return direction * projected_translation(start, current)
    if kind == "turn":
        return direction * angle_delta(start["yaw"], current["yaw"])
    raise ValueError(f"unknown square segment kind: {kind}")


def compare_world_odom(start_world, end_world, start_odom, end_odom):
    world_dx = end_world["position"][0] - start_world["position"][0]
    world_dy = end_world["position"][1] - start_world["position"][1]
    odom_dx = end_odom["position"][0] - start_odom["position"][0]
    odom_dy = end_odom["position"][1] - start_odom["position"][1]
    world_yaw = angle_delta(pose_yaw(start_world), pose_yaw(end_world))
    odom_yaw = angle_delta(start_odom["yaw"], end_odom["yaw"])
    return {"translation_m": math.hypot(world_dx-odom_dx, world_dy-odom_dy),
            "yaw_rad": abs(angle_delta(world_yaw, odom_yaw))}


def drive_pose_as_footprint(odom):
    """Convert axle-origin odometry to the chassis-centered base_footprint pose."""
    yaw = odom["yaw"]
    offset = -DRIVE_FRAME_X
    return {**odom, "position": (odom["position"][0]+offset*math.cos(yaw),
                                  odom["position"][1]+offset*math.sin(yaw),
                                  odom["position"][2]),
            "child_frame_id": "base_footprint"}


def integrate_imu_yaw(samples):
    ordered = sorted(samples, key=lambda sample: sample["stamp_ns"])
    total = 0.0
    for before, after in zip(ordered, ordered[1:]):
        dt = (after["stamp_ns"]-before["stamp_ns"])/1_000_000_000
        if dt < 0:
            raise ValueError("IMU simulation timestamp rewound within a segment")
        total += (before["angular_z"]+after["angular_z"])*0.5*dt
    return total


def world_control_command(paused):
    value = "true" if paused else "false"
    return ["ign", "service", "-s", f"/world/{WORLD}/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean", "--timeout", "3000",
            "-r", f"pause: {value}"]


def assert_tf_ownership(node, report):
    names = [item.node_name.lstrip("/") for item in node.get_publishers_info_by_topic("/tf")]
    counts = {name: names.count(name) for name in set(names)}
    if counts.get("ekf_filter_node") != 1:
        raise RuntimeError(f"expected exactly one /tf writer from ekf_filter_node: {counts}")
    unexpected = set(names) - {"ekf_filter_node", "robot_state_publisher"}
    if unexpected or "robot_state_publisher" not in names:
        raise RuntimeError(f"unexpected /tf ownership endpoints: {counts}")
    report["tf_publishers"] = sorted(names)
    report["odom_tf_owner"] = "ekf_filter_node"


def fresh_snapshot(checks):
    odom = checks.odom_snapshot()
    if any(odom[key] is None for key in ("unfiltered", "filtered")) or not checks.tf_samples:
        raise ValueError("both odometry streams and odom->base_footprint TF are required")
    return odom, checks.tf_samples[-1]


def run_square_segment(checks, guard, report, kind, index):
    linear = LINEAR_LIMIT if kind == "straight" else 0.0
    angular = ANGULAR_LIMIT if kind == "turn" else 0.0
    target, maximum, timeout = SIDE_METERS, 0.25, 18.0
    if kind == "turn":
        target, maximum, timeout = TURN_RADIANS, 1.90, 36.0

    start_world = read_model_pose(time.monotonic()+7)
    start_odom, _ = fresh_snapshot(checks)
    begin, deadline = time.monotonic(), time.monotonic()+timeout
    next_publish, next_guard, progress = begin, begin, 0.0
    while progress < target:
        now = time.monotonic()
        if now >= deadline:
            raise TimeoutError(f"square {kind} {index}: target not reached before {timeout}s")
        if now >= next_guard:
            guard.check()
            next_guard = now+0.5
        if now >= next_publish:
            checks.publish(linear, angular)
            next_publish = now+0.1
        checks.spin(min(0.05, max(0.0, deadline-now)))
        current = checks.odom_samples["filtered"][-1]
        if current["time"] >= begin:
            progress = odom_progress(kind, start_odom["filtered"], current)
            if progress > maximum:
                raise ValueError(f"square {kind} {index}: safety progress limit exceeded ({progress:.4f})")

    command_end = time.monotonic()
    checks.stop(0.35)
    checks.pump(0.25)
    end_world = read_model_pose(time.monotonic()+7)
    end_odom, end_tf = fresh_snapshot(checks)
    if kind == "straight":
        gt_delta = projected_translation(start_world, end_world)
        raw_delta = projected_translation(start_odom["unfiltered"], end_odom["unfiltered"])
        filtered_delta = projected_translation(start_odom["filtered"], end_odom["filtered"])
        min_gt, max_gt = 0.08, 0.25
        expected_wheels = lambda values: all(value > 0.05 for value in values.values())
    else:
        gt_delta = angle_delta(pose_yaw(start_world), pose_yaw(end_world))
        raw_delta = angle_delta(start_odom["unfiltered"]["yaw"], end_odom["unfiltered"]["yaw"])
        filtered_delta = angle_delta(start_odom["filtered"]["yaw"], end_odom["filtered"]["yaw"])
        min_gt, max_gt = 1.10, 1.90
        expected_wheels = lambda values: values["left_wheel_joint"] < -0.05 and values["right_wheel_joint"] > 0.05
    if gt_delta < min_gt or gt_delta > max_gt:
        raise ValueError(f"square {kind} {index}: ground-truth movement out of bounds ({gt_delta:.4f})")
    min_raw_delta = target*0.70 if kind == "straight" else 0.75
    if raw_delta < min_raw_delta:
        raise ValueError(f"square {kind} {index}: raw odometry direction/magnitude invalid ({raw_delta:.4f})")
    if filtered_delta < target*0.90:
        raise ValueError(f"square {kind} {index}: EKF odometry did not reach target ({filtered_delta:.4f})")
    raw_start_footprint = drive_pose_as_footprint(start_odom["unfiltered"])
    raw_end_footprint = drive_pose_as_footprint(end_odom["unfiltered"])
    raw_error = compare_world_odom(start_world, end_world,
                                   raw_start_footprint, raw_end_footprint)
    filtered_error = compare_world_odom(start_world, end_world,
                                        start_odom["filtered"], end_odom["filtered"])
    raw_warnings = []
    if raw_error["translation_m"] > 0.07 or raw_error["yaw_rad"] > 0.20:
        raw_warnings.append(f"raw odometry drift: {raw_error}")
    tolerance_errors = []
    if filtered_error["translation_m"] > 0.10 or filtered_error["yaw_rad"] > 0.30:
        tolerance_errors.append(f"EKF odometry drift: {filtered_error}")
    tf_translation = math.dist(end_odom["filtered"]["position"], end_tf["position"])
    tf_yaw = abs(angle_delta(end_odom["filtered"]["yaw"], end_tf["yaw"]))
    if tf_translation > 0.03 or tf_yaw > 0.10:
        raise ValueError(f"square {kind} {index}: EKF/TF mismatch ({tf_translation:.4f}m, {tf_yaw:.4f}rad)")

    joints = [sample for sample in checks.joint_samples if begin <= sample["time"] <= command_end]
    if len(joints) < 3:
        raise ValueError(f"square {kind} {index}: insufficient wheel JointState samples")
    wheel_means = {joint: sum(item["velocities"][joint] for item in joints)/len(joints)
                   for joint in JOINTS}
    if not expected_wheels(wheel_means):
        raise ValueError(f"square {kind} {index}: wheel directions incorrect: {wheel_means}")
    imu = [sample for sample in checks.imu_samples if begin <= sample["time"] <= command_end]
    if len(imu) < 3:
        raise ValueError(f"square {kind} {index}: insufficient IMU angular-velocity samples")
    imu_metrics = {"samples": len(imu),
                   "mean_angular_z_rps": sum(item["angular_z"] for item in imu)/len(imu),
                   "integrated_yaw_rad": integrate_imu_yaw(imu)}
    settled = [sample for sample in checks.joint_samples if sample["time"] >= time.monotonic()-0.25]
    if not settled or max(abs(value) for value in settled[-1]["velocities"].values()) > 0.5:
        raise ValueError(f"square {kind} {index}: wheel state did not settle after stop")
    result = {"name": f"{kind}_{index}", "kind": kind, "target": target,
              "odom_progress": progress, "ground_truth_delta": gt_delta,
              "unfiltered_odom_delta": raw_delta, "filtered_odom_delta": filtered_delta,
              "ground_truth_vs_unfiltered_error": raw_error,
              "ground_truth_vs_filtered_error": filtered_error,
              "raw_drift_warnings": raw_warnings,
              "tolerance_errors": tolerance_errors,
              "odom_tf_error": {"translation_m": tf_translation, "yaw_rad": tf_yaw},
              "mean_wheel_velocity_rad_s": wheel_means, "imu": imu_metrics,
              "wall_seconds": command_end-begin, "pass": not tolerance_errors}
    report.setdefault("square_segments", []).append(result)
    report.setdefault("square_tolerance_errors", []).extend(
        f"{result['name']}: {error}" for error in tolerance_errors)
    report.setdefault("raw_drift_warnings", []).extend(
        f"{result['name']}: {warning}" for warning in raw_warnings)


def run_square(checks, guard, report):
    start_world = read_model_pose(time.monotonic()+7)
    start_odom, _ = fresh_snapshot(checks)
    for index in range(1, 5):
        run_square_segment(checks, guard, report, "straight", index)
        run_square_segment(checks, guard, report, "turn", index)
    end_world = read_model_pose(time.monotonic()+7)
    end_odom, _ = fresh_snapshot(checks)
    raw_start_footprint = drive_pose_as_footprint(start_odom["unfiltered"])
    raw_end_footprint = drive_pose_as_footprint(end_odom["unfiltered"])
    closure = {
        "ground_truth_position_m": math.dist(start_world["position"][:2], end_world["position"][:2]),
        "unfiltered_odom_position_m": math.dist(raw_start_footprint["position"][:2],
                                                 raw_end_footprint["position"][:2]),
        "filtered_odom_position_m": math.dist(start_odom["filtered"]["position"][:2],
                                               end_odom["filtered"]["position"][:2]),
        "ground_truth_yaw_rad": abs(angle_delta(pose_yaw(start_world), pose_yaw(end_world))),
        "unfiltered_odom_yaw_rad": abs(angle_delta(start_odom["unfiltered"]["yaw"], end_odom["unfiltered"]["yaw"])),
        "filtered_odom_yaw_rad": abs(angle_delta(start_odom["filtered"]["yaw"], end_odom["filtered"]["yaw"]))}
    closure_errors = []
    if max(closure[key] for key in ("ground_truth_position_m", "filtered_odom_position_m")) > 0.30:
        closure_errors.append(f"square endpoint position closure exceeded 0.30m: {closure}")
    if max(closure[key] for key in ("ground_truth_yaw_rad", "filtered_odom_yaw_rad")) > 0.50:
        closure_errors.append(f"square endpoint yaw closure exceeded 0.50rad: {closure}")
    report["square_closure"] = closure
    report["square_closure_errors"] = closure_errors


def run_watchdog_and_recovery(checks, guard, report):
    guard.check()
    checks.assert_publishers_isolated(NODE_NAME)
    command_index = len(checks.command_echoes)
    begin, end = time.monotonic(), time.monotonic()+1.5
    next_publish, next_guard, last_nonzero = begin, begin, None
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_guard:
            guard.check()
            next_guard = now+0.5
        if now >= next_publish:
            checks.publish(LINEAR_LIMIT, 0.0)
            last_nonzero = time.monotonic()
            next_publish = now+0.1
        checks.spin(min(0.05, max(0.0, end-time.monotonic())))

    deadline, echo = time.monotonic()+1.2, None
    while time.monotonic() < deadline and echo is None:
        checks.spin(min(0.05, max(0.0, deadline-time.monotonic())))
        echo = next((item for item in checks.command_echoes[command_index:]
                     if item["zero"] and item["time"] >= last_nonzero+0.15), None)
    if echo is None:
        raise TimeoutError("watchdog emitted no zero within 1.2s after command loss")
    latency = echo["time"]-last_nonzero
    if not 0.15 <= latency <= 1.2:
        raise ValueError(f"watchdog zero latency outside [0.15, 1.2]s: {latency:.4f}s")

    settle_deadline, calm = time.monotonic()+1.5, 0
    while time.monotonic() < settle_deadline and calm < 3:
        checks.spin(min(0.05, max(0.0, settle_deadline-time.monotonic())))
        if checks.joint_samples and max(abs(value) for value in
                                        checks.joint_samples[-1]["velocities"].values()) <= 0.5:
            calm += 1
        else:
            calm = 0
    if calm < 3:
        raise TimeoutError("wheel JointStates did not settle after watchdog zero")

    stopped_a = read_model_pose(time.monotonic()+7)
    checks.pump(0.4)
    stopped_b = read_model_pose(time.monotonic()+7)
    residual = math.dist(stopped_a["position"][:2], stopped_b["position"][:2])
    if residual > 0.02:
        raise ValueError(f"robot continued moving after watchdog stop: {residual:.4f}m")

    recovery_start = read_model_pose(time.monotonic()+7)
    recovery_odom, _ = fresh_snapshot(checks)
    recovery_begin, recovery_end = time.monotonic(), time.monotonic()+1.5
    next_publish = recovery_begin
    while time.monotonic() < recovery_end:
        now = time.monotonic()
        if now >= next_publish:
            checks.publish(0.05, 0.0)
            next_publish = now+0.1
        checks.spin(min(0.05, max(0.0, recovery_end-time.monotonic())))
    checks.stop(0.4)
    checks.pump(0.25)
    recovery_finish = read_model_pose(time.monotonic()+7)
    recovery_odom_end, _ = fresh_snapshot(checks)
    world_delta = projected_translation(recovery_start, recovery_finish)
    odom_delta = projected_translation(recovery_odom["unfiltered"], recovery_odom_end["unfiltered"])
    if world_delta < 0.015 or odom_delta < 0.015:
        raise ValueError(f"fresh command did not recover motion: world={world_delta:.4f}, odom={odom_delta:.4f}")
    report["watchdog_recovery"] = {
        "last_nonzero_command_time": last_nonzero, "zero_echo": echo,
        "zero_latency_wall_seconds": latency, "residual_motion_after_stop_m": residual,
        "recovery_world_delta_m": world_delta, "recovery_unfiltered_odom_delta_m": odom_delta,
        "pass": True}


def pause_aware_pump(checks, guard, duration):
    import rclpy
    deadline = time.monotonic()+duration
    next_guard = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_guard:
            guard.check()
            next_guard = now+0.5
        rclpy.spin_once(checks.node, timeout_sec=min(0.05, max(0.0, deadline-now)))
        if checks.error:
            raise ValueError(checks.error)


def set_world_paused(paused):
    result = subprocess.run(world_control_command(paused), capture_output=True,
                            text=True, check=False, timeout=7)
    if result.returncode or "true" not in result.stdout.lower():
        raise RuntimeError(f"Fortress world {'pause' if paused else 'resume'} request failed: "
                           f"rc={result.returncode}, {result.stderr[:500]}, {result.stdout[:300]}")


def run_pause_watchdog_test(checks, guard, report):
    guard.check()
    checks.assert_publishers_isolated(NODE_NAME)
    command_index = len(checks.command_echoes)
    checks.publish(LINEAR_LIMIT, 0.0)
    last_nonzero = time.monotonic()
    paused = False
    paused_advances = None
    try:
        paused = True
        set_world_paused(True)
        pause_confirmed = time.monotonic()
        pause_aware_pump(checks, guard, 0.25)
        frozen_advances = checks.clock_advances
        pause_aware_pump(checks, guard, 0.5)
        paused_advances = checks.clock_advances-frozen_advances
        if paused_advances:
            raise RuntimeError("/clock continued advancing while Fortress reported paused")

        deadline, zero_echo = time.monotonic()+1.2, None
        while time.monotonic() < deadline and zero_echo is None:
            pause_aware_pump(checks, guard, min(0.05, max(0.0, deadline-time.monotonic())))
            zero_echo = next((item for item in checks.command_echoes[command_index:]
                              if item["zero"] and item["time"] >= pause_confirmed
                              and item["time"] >= last_nonzero+0.15), None)
        if zero_echo is None:
            raise TimeoutError("wall-clock watchdog did not emit zero while simulation was paused")
        pause_latency = zero_echo["time"]-last_nonzero
        if pause_latency > 1.2:
            raise ValueError(f"pause watchdog latency exceeded 1.2s: {pause_latency:.4f}s")
    finally:
        if paused:
            set_world_paused(False)
            paused = False

    resume_at = time.monotonic()
    old_advances = checks.clock_advances
    resume_deadline = resume_at+5.0
    while time.monotonic() < resume_deadline and checks.clock_advances == old_advances:
        pause_aware_pump(checks, guard, min(0.05, max(0.0, resume_deadline-time.monotonic())))
    if checks.clock_advances == old_advances:
        raise TimeoutError("/clock did not resume after Fortress unpause")
    checks.pump(0.5)
    stale = [item for item in checks.command_echoes[command_index:]
             if item["time"] >= resume_at and not item["zero"]]
    if stale:
        raise ValueError(f"stale nonzero `/cmd_vel` echo after resume: {stale[0]}")
    stationary_a = read_model_pose(time.monotonic()+7)
    checks.pump(0.4)
    stationary_b = read_model_pose(time.monotonic()+7)
    post_resume_motion = math.dist(stationary_a["position"][:2], stationary_b["position"][:2])
    if post_resume_motion > 0.02:
        raise ValueError(f"robot moved after resume without a fresh command: {post_resume_motion:.4f}m")
    report["pause_watchdog"] = {
        "clock_advances_while_paused": paused_advances,
        "zero_echo": zero_echo, "zero_latency_wall_seconds": pause_latency,
        "clock_resumed": True, "stale_nonzero_echoes_after_resume": len(stale),
        "motion_without_fresh_command_m": post_resume_motion, "pass": True}


def run_probe(args, report, persist):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from verify_camera_gui import OwnershipGuard

    guard = OwnershipGuard(args.supervisor_pid, partition_prefix="linobot2_g3_")
    guard.check()
    node, initialized, checks = None, False, None
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node(NODE_NAME)
        types = {"Twist": Twist, "Odometry": Odometry, "Clock": Clock,
                 "JointState": JointState, "TFMessage": TFMessage}
        checks = MotionChecks(node, guard, report, persist, types)
        from sensor_msgs.msg import Imu
        checks.imu_samples = []

        def on_imu(msg):
            try:
                value = float(msg.angular_velocity.z)
                stamp_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
                if not math.isfinite(value):
                    raise ValueError("nonfinite IMU angular velocity")
                checks.imu_samples.append({"time": time.monotonic(), "stamp_ns": stamp_ns,
                                           "angular_z": value})
            except (AttributeError, TypeError, ValueError) as exc:
                checks.error = checks.error or str(exc)

        node.create_subscription(Imu, "/imu/data", on_imu, 10)
        checks.warmup()
        checks.assert_publishers_isolated(NODE_NAME)
        assert_tf_ownership(node, report)
        report["initial"] = {"joint_state": checks.joint_samples[-1],
                             "odom_unfiltered": checks.odom_samples["unfiltered"][-1],
                             "odom_filtered": checks.odom_samples["filtered"][-1],
                             "clock_advances": checks.clock_advances}
        report["limits"] = {"linear_x_mps": LINEAR_LIMIT, "angular_z_rps": ANGULAR_LIMIT,
                            "square_side_m": SIDE_METERS, "turn_target_rad": TURN_RADIANS,
                    "raw_odom_child_frame": "base_drive",
                    "raw_to_footprint_offset_x_m": DRIVE_FRAME_X,
                    "hardware_topics_or_devices": "none"}
        persist()
        run_square(checks, guard, report)
        report["square_complete"] = True
        report["square_pass"] = (not report.get("square_tolerance_errors")
                                  and not report.get("square_closure_errors"))
        report["errors"].extend(report.get("square_tolerance_errors", []))
        report["errors"].extend(report.get("square_closure_errors", []))
        run_pause_watchdog_test(checks, guard, report)
        report["simulation_pause_tested"] = True
        run_watchdog_and_recovery(checks, guard, report)
        guard.check()
        report["clock_advances"] = checks.clock_advances
        report["joint_state_samples"] = len(checks.joint_samples)
        report["odom_unfiltered_samples"] = len(checks.odom_samples["unfiltered"])
        report["odom_filtered_samples"] = len(checks.odom_samples["filtered"])
        report["tf_samples"] = len(checks.tf_samples)
        report["imu_samples"] = len(checks.imu_samples)
        report["sensor_failure_tested"] = False
        report["pass"] = report["square_complete"] and report["square_pass"]
    finally:
        if checks is not None:
            try:
                checks.stop(0.4)
                report.setdefault("safety", {})["final_zero_published"] = True
            except Exception as exc:
                report.setdefault("safety", {})["final_zero_error"] = f"{type(exc).__name__}: {exc}"
                report["pass"] = False
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if initialized and rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--supervisor-pid", type=int, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    partition = os.environ.get("IGN_PARTITION", "")
    if args.supervisor_pid <= 0 or os.environ.get("ROS_DOMAIN_ID") != "42" \
            or not partition.startswith("linobot2_g3_") or partition == "linobot2_g3_":
        parser.error("require a positive supervisor, ROS_DOMAIN_ID=42 and unique linobot2_g3_ partition")
    start = time.monotonic()
    report = {"pass": False, "status": "RUNNING", "errors": [],
              "scope": "G3_SIMULATION_ONLY", "world": WORLD, "model": MODEL,
              "supervisor_pid": args.supervisor_pid, "domain_id": "42",
              "partition": partition, "simulation_pause_tested": False,
              "sensor_failure_tested": False, "simulation_restart_tested": False}
    persist = lambda: write_report(args.report, report)
    previous = {}

    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"received signal {signum}")

    try:
        persist()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        run_probe(args, report, persist)
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        report["pass"] = False
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        report["elapsed_seconds"] = time.monotonic()-start
        report["status"] = "PASS" if report["pass"] else "FAIL"
        try:
            persist()
        except (OSError, ValueError, TypeError) as exc:
            report["pass"] = False
            report["errors"].append(f"cannot save JSON report: {exc}")
    print(f"{report['status']} [{report['scope']}]: "
          + (str(args.report) if report["pass"] else "; ".join(report["errors"])))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
