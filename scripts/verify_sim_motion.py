#!/usr/bin/env python3
"""Bounded, simulation-only 2WD joint/odometry/command verification for G2."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


WORLD = "fortress_test"
MODEL = "linorobot2_2wd"
POSE_TOPIC = f"/world/{WORLD}/dynamic_pose/info"
JOINTS = ("left_wheel_joint", "right_wheel_joint")
NODE_NAME = "linorobot2_g2_motion_probe"


def yaw_from_quaternion(quaternion):
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError("pose quaternion must contain four finite values")
    norm = math.sqrt(sum(v*v for v in quaternion))
    if abs(norm - 1.0) > 0.01:
        raise ValueError("pose quaternion is not unit length")
    x, y, z, w = (v/norm for v in quaternion)
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def angle_delta(start, end):
    return math.atan2(math.sin(end-start), math.cos(end-start))


def projected_translation(start, end):
    yaw = yaw_from_quaternion(start["orientation"])
    dx = end["position"][0] - start["position"][0]
    dy = end["position"][1] - start["position"][1]
    return dx*math.cos(yaw) + dy*math.sin(yaw)


def parse_model_pose_stream(text):
    decoder, offset, samples = json.JSONDecoder(), 0, []
    while True:
        while offset < len(text) and text[offset].isspace():
            offset += 1
        if offset == len(text):
            break
        document, offset = decoder.raw_decode(text, offset)
        entries = document.get("pose") if isinstance(document, dict) else None
        if not isinstance(entries, list):
            raise ValueError("Gazebo pose message must contain a pose list")
        matches = [entry for entry in entries
                   if isinstance(entry, dict) and entry.get("name") == MODEL]
        if len(matches) != 1:
            raise ValueError(f"need exactly one exact {MODEL} entry per Gazebo pose message")
        item = matches[0]
        position, orientation = item.get("position"), item.get("orientation")
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            raise ValueError("Gazebo model pose lacks position or orientation")
        pose = {
            "position": tuple(float(position.get(axis, 0.0)) for axis in "xyz"),
            "orientation": tuple(float(orientation.get(axis, 0.0)) for axis in "xyzw"),
        }
        if not all(math.isfinite(value) for values in pose.values() for value in values):
            raise ValueError("Gazebo model pose contains nonfinite values")
        yaw_from_quaternion(pose["orientation"])
        samples.append(pose)
    if not samples:
        raise ValueError("Gazebo produced no model pose messages")
    return samples[-1]


def read_model_pose(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError("wall-clock deadline exceeded before Gazebo pose query")
    result = subprocess.run(
        ["ign", "topic", "-e", "-t", POSE_TOPIC, "-n", "1", "--json-output"],
        capture_output=True, text=True, check=False, timeout=min(5.0, remaining))
    if result.returncode:
        raise RuntimeError(f"Gazebo model-pose query failed: {result.stderr[:1000]}")
    return parse_model_pose_stream(result.stdout)


def finite_vector(values, label):
    converted = tuple(float(value) for value in values)
    if not converted or not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{label} must contain finite values")
    return converted


def write_report(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class MotionChecks:
    def __init__(self, node, guard, report, persist, types):
        self.node, self.guard, self.report, self.persist = node, guard, report, persist
        self.Twist = types["Twist"]
        self.command_pub = node.create_publisher(self.Twist, "/cmd_vel", 10)
        self.joint_samples = []
        self.odom_samples = {"unfiltered": [], "filtered": []}
        self.tf_samples = []
        self.command_echoes = []
        self.clock_ns = None
        self.clock_advances = 0
        self.clock_started_at = time.monotonic()
        self.clock_progress_at = self.clock_started_at
        self.error = None
        node.create_subscription(types["Clock"], "/clock", self.on_clock, 10)
        node.create_subscription(types["JointState"], "/joint_states", self.on_joints, 10)
        node.create_subscription(types["Odometry"], "/odom/unfiltered",
                                 lambda msg: self.on_odom("unfiltered", msg), 10)
        node.create_subscription(types["Odometry"], "/odom",
                                 lambda msg: self.on_odom("filtered", msg), 10)
        node.create_subscription(types["TFMessage"], "/tf", self.on_tf, 10)
        node.create_subscription(types["Twist"], "/cmd_vel", self.on_command_echo, 10)

    def on_clock(self, msg):
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        now = time.monotonic()
        if self.clock_ns is not None:
            if stamp < self.clock_ns:
                self.error = self.error or "simulation clock rewound"
            elif stamp > self.clock_ns:
                self.clock_advances += 1
                self.clock_progress_at = now
        self.clock_ns = stamp

    def on_joints(self, msg):
        try:
            if len(set(msg.name)) != len(msg.name):
                raise ValueError("JointState contains duplicate joint names")
            if not set(JOINTS).issubset(msg.name):
                raise ValueError(f"JointState lacks required joints {JOINTS}: {msg.name}")
            if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
                raise ValueError("JointState name/position/velocity lengths differ")
            index = {name: i for i, name in enumerate(msg.name)}
            positions = {name: float(msg.position[index[name]]) for name in JOINTS}
            velocities = {name: float(msg.velocity[index[name]]) for name in JOINTS}
            if not all(math.isfinite(value) for value in (*positions.values(), *velocities.values())):
                raise ValueError("JointState contains nonfinite wheel values")
            self.joint_samples.append({"time": time.monotonic(),
                                      "positions": positions, "velocities": velocities})
        except (AttributeError, TypeError, ValueError) as exc:
            self.error = self.error or str(exc)

    @staticmethod
    def odom_values(msg):
        pose = msg.pose.pose
        position = finite_vector((pose.position.x, pose.position.y, pose.position.z), "odometry position")
        orientation = finite_vector((pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w), "odometry quaternion")
        yaw = yaw_from_quaternion(orientation)
        twist = msg.twist.twist
        linear = finite_vector((twist.linear.x, twist.linear.y, twist.linear.z), "odometry linear velocity")
        angular = finite_vector((twist.angular.x, twist.angular.y, twist.angular.z), "odometry angular velocity")
        return {"time": time.monotonic(), "position": position,
                "orientation": orientation, "yaw": yaw, "linear": linear,
                "angular": angular, "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id}

    def on_odom(self, key, msg):
        try:
            values = self.odom_values(msg)
            expected_child = "base_drive" if key == "unfiltered" else "base_footprint"
            if values["frame_id"].lstrip("/") != "odom" or values["child_frame_id"].lstrip("/") != expected_child:
                raise ValueError(f"/odom {key} frame contract mismatch: "
                                 f"{values['frame_id']} -> {values['child_frame_id']}")
            self.odom_samples[key].append(values)
        except (AttributeError, TypeError, ValueError) as exc:
            self.error = self.error or str(exc)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id.lstrip("/") == "odom"
                    and transform.child_frame_id.lstrip("/") == "base_footprint"):
                try:
                    translation = finite_vector((transform.transform.translation.x,
                                                 transform.transform.translation.y,
                                                 transform.transform.translation.z), "odom TF translation")
                    orientation = finite_vector((transform.transform.rotation.x,
                                                 transform.transform.rotation.y,
                                                 transform.transform.rotation.z,
                                                 transform.transform.rotation.w), "odom TF quaternion")
                    self.tf_samples.append({"time": time.monotonic(), "position": translation,
                                            "orientation": orientation,
                                            "yaw": yaw_from_quaternion(orientation)})
                except (AttributeError, TypeError, ValueError) as exc:
                    self.error = self.error or str(exc)

    def on_command_echo(self, msg):
        values = (float(msg.linear.x), float(msg.angular.z))
        if not all(math.isfinite(value) for value in values):
            self.error = self.error or "nonfinite /cmd_vel echo"
            return
        self.command_echoes.append({"time": time.monotonic(), "linear_x": values[0],
                                    "angular_z": values[1],
                                    "zero": values == (0.0, 0.0)})

    def ensure_ok(self):
        if self.error:
            raise ValueError(self.error)
        now = time.monotonic()
        if self.clock_ns is None:
            if now-self.clock_started_at > 5:
                raise ValueError("simulation clock is absent")
            return
        if not self.clock_advances:
            if now-self.clock_started_at > 5:
                raise ValueError("simulation clock is not advancing")
            return
        if now-self.clock_progress_at > 3:
            raise ValueError("simulation clock is absent or stopped")

    def spin(self, timeout=0.05):
        import rclpy
        if not rclpy.ok():
            raise RuntimeError("ROS context stopped during motion verification")
        rclpy.spin_once(self.node, timeout_sec=timeout)
        self.ensure_ok()

    def publish(self, linear_x=0.0, angular_z=0.0):
        msg = self.Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.command_pub.publish(msg)

    def stop(self, duration=0.4):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.publish()
            self.spin(min(0.05, max(0.0, end-time.monotonic())))

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.spin(min(0.05, max(0.0, end-time.monotonic())))

    def warmup(self, timeout=25.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (len(self.joint_samples) >= 3 and self.odom_samples["unfiltered"]
                    and self.odom_samples["filtered"] and self.tf_samples and self.clock_advances):
                self.ensure_ok()
                return
            self.spin(min(0.1, max(0.0, deadline-time.monotonic())))
        raise TimeoutError("required wheel JointState, odometry, TF and /clock not ready")

    def assert_publishers_isolated(self, probe_node_name=NODE_NAME):
        names = {item.node_name.lstrip("/")
                 for item in self.node.get_publishers_info_by_topic("/cmd_vel")}
        allowed = {probe_node_name, "command_timeout"}
        if "command_timeout" not in names or probe_node_name not in names or names - allowed:
            raise RuntimeError(f"unexpected /cmd_vel publishers in isolated simulation: {sorted(names)}")
        self.report["cmd_vel_publishers"] = sorted(names)

    def odom_snapshot(self):
        return {key: values[-1] if values else None for key, values in self.odom_samples.items()}

    def verify_segment(self, name, linear_x, angular_z, duration=2.0, watchdog=False):
        self.guard.check()
        self.assert_publishers_isolated()
        start_pose = read_model_pose(time.monotonic()+7)
        start_odom = self.odom_snapshot()
        start_tf = self.tf_samples[-1] if self.tf_samples else None
        if any(start_odom[key] is None for key in ("unfiltered", "filtered")) or start_tf is None:
            raise ValueError(f"{name}: baseline odometry or odom->base_footprint TF missing")
        joint_index = len(self.joint_samples)
        odom_indices = {key: len(values) for key, values in self.odom_samples.items()}
        cmd_index = len(self.command_echoes)
        began = time.monotonic()
        end = began + duration
        next_publish = began
        next_guard = began
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_guard:
                self.guard.check()
                next_guard = now + 0.5
            if now >= next_publish:
                self.publish(linear_x, angular_z)
                next_publish = now + 0.1
            self.spin(min(0.05, max(0.0, end-time.monotonic())))
        command_end = time.monotonic()
        watchdog_echo = None
        if watchdog:
            # Deliberately stop publishing without sending a zero. The only
            # permitted other publisher is command_timeout, which must emit it.
            deadline = command_end + 1.5
            while time.monotonic() < deadline and watchdog_echo is None:
                self.spin(min(0.05, max(0.0, deadline-time.monotonic())))
                watchdog_echo = next((item for item in self.command_echoes[cmd_index:]
                                      if item["zero"] and item["time"] >= command_end+0.15), None)
            if watchdog_echo is None:
                raise TimeoutError("command_timeout did not emit zero within 1.5 wall seconds")
            self.pump(0.4)
        else:
            self.stop(0.4)
            self.pump(0.2)
        finish_pose = read_model_pose(time.monotonic()+7)
        finish_odom = self.odom_snapshot()
        finish_tf = self.tf_samples[-1] if self.tf_samples else None
        end_time = time.monotonic()
        active_joints = [sample for sample in self.joint_samples[joint_index:]
                         if began <= sample["time"] <= command_end]
        if len(active_joints) < 3:
            raise ValueError(f"{name}: only {len(active_joints)} wheel-state samples during command")
        wheel_means = {joint: sum(x["velocities"][joint] for x in active_joints)/len(active_joints)
                       for joint in JOINTS}
        direction = 1 if linear_x > 0 else -1 if linear_x < 0 else 0
        turn = 1 if angular_z > 0 else -1 if angular_z < 0 else 0
        if direction:
            if any(value*direction <= 0.05 for value in wheel_means.values()):
                raise ValueError(f"{name}: wheel directions/velocities incorrect: {wheel_means}")
            displacement = projected_translation(start_pose, finish_pose)
            if displacement*direction < 0.02:
                raise ValueError(f"{name}: Gazebo world-pose displacement has wrong/small sign: {displacement:.5f}m")
            if abs(displacement) > 0.4:
                raise ValueError(f"{name}: bounded translation exceeded 0.4m: {displacement:.5f}m")
            odom_motion = {}
            for key in self.odom_samples:
                samples = self.odom_samples[key][odom_indices[key]:]
                moving = [sample for sample in samples if began <= sample["time"] <= command_end]
                if len(moving) < 2:
                    raise ValueError(f"{name}: insufficient {key} odometry during motion")
                velocity = sum(sample["linear"][0] for sample in moving)/len(moving)
                if velocity*direction <= 0.005:
                    raise ValueError(f"{name}: {key} linear odometry direction incorrect: {velocity:.5f}m/s")
                pose_delta = projected_translation(start_odom[key], finish_odom[key])
                if pose_delta*direction < 0.01:
                    raise ValueError(f"{name}: {key} odometry pose moved wrong/too little: {pose_delta:.5f}m")
                odom_motion[key] = {"mean_linear_x_mps": velocity,
                                    "signed_pose_delta_m": pose_delta, "samples": len(moving)}
            tf_delta = projected_translation(start_tf, finish_tf)
            if tf_delta*direction < 0.01:
                raise ValueError(f"{name}: odom TF moved wrong/too little: {tf_delta:.5f}m")
            motion_value = displacement
        else:
            if wheel_means["left_wheel_joint"] * wheel_means["right_wheel_joint"] >= -0.0025:
                raise ValueError(f"{name}: wheels did not rotate in opposite directions: {wheel_means}")
            yaw_change = angle_delta(yaw_from_quaternion(start_pose["orientation"]),
                                     yaw_from_quaternion(finish_pose["orientation"]))
            if yaw_change*turn < 0.08:
                raise ValueError(f"{name}: Gazebo yaw has wrong/small sign: {yaw_change:.5f}rad")
            if abs(yaw_change) > 1.0:
                raise ValueError(f"{name}: bounded yaw exceeded 1.0rad: {yaw_change:.5f}rad")
            odom_motion = {}
            for key in self.odom_samples:
                moving = [sample for sample in self.odom_samples[key][odom_indices[key]:]
                          if began <= sample["time"] <= command_end]
                if len(moving) < 2:
                    raise ValueError(f"{name}: insufficient {key} odometry during rotation")
                velocity = sum(sample["angular"][2] for sample in moving)/len(moving)
                if velocity*turn <= 0.01:
                    raise ValueError(f"{name}: {key} yaw-rate direction incorrect: {velocity:.5f}rad/s")
                pose_delta = angle_delta(start_odom[key]["yaw"], finish_odom[key]["yaw"])
                if pose_delta*turn < 0.05:
                    raise ValueError(f"{name}: {key} odometry yaw moved wrong/too little: {pose_delta:.5f}rad")
                odom_motion[key] = {"mean_angular_z_rps": velocity,
                                    "signed_yaw_delta_rad": pose_delta, "samples": len(moving)}
            tf_delta = angle_delta(start_tf["yaw"], finish_tf["yaw"])
            if tf_delta*turn < 0.05:
                raise ValueError(f"{name}: odom TF yaw moved wrong/too little: {tf_delta:.5f}rad")
            motion_value = yaw_change
        if finish_odom["filtered"] is None or finish_odom["unfiltered"] is None or finish_tf is None:
            raise ValueError(f"{name}: missing odometry or odom->base_footprint TF")
        tf_translation_error = math.dist(finish_odom["filtered"]["position"], finish_tf["position"])
        tf_yaw_error = abs(angle_delta(finish_odom["filtered"]["yaw"], finish_tf["yaw"]))
        if tf_translation_error > 0.03 or tf_yaw_error > 0.10:
            raise ValueError(f"{name}: EKF odom/TF disagree ({tf_translation_error:.4f}m, {tf_yaw_error:.4f}rad)")
        settled = [sample for sample in self.joint_samples if sample["time"] >= end_time-0.2]
        if not settled:
            raise ValueError(f"{name}: no wheel states after stopping")
        wheel_final = settled[-1]["velocities"]
        if max(abs(value) for value in wheel_final.values()) > 0.5:
            raise ValueError(f"{name}: wheel velocity failed to settle after stop: {wheel_final}")
        result = {
            "name": name, "command": {"linear_x_mps": linear_x, "angular_z_rps": angular_z,
                                        "duration_wall_seconds": duration},
            "world_pose_start": start_pose, "world_pose_end": finish_pose,
            "signed_motion": motion_value, "wheel_mean_velocity_rad_s": wheel_means,
            "odom_motion": odom_motion, "odom_tf_error": {
                "translation_m": tf_translation_error, "yaw_rad": tf_yaw_error},
            "wheel_velocity_after_stop_rad_s": wheel_final,
            "joint_samples_during_command": len(active_joints),
            "watchdog_zero_echo": watchdog_echo,
            "pass": True,
        }
        self.report.setdefault("segments", []).append(result)
        self.persist()
        return result


def run_motion_probe(args, report, persist):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from verify_camera_gui import OwnershipGuard

    guard = OwnershipGuard(args.supervisor_pid, partition_prefix="linobot2_g2_")
    guard.check()
    node = None
    initialized = False
    checks = None
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node(NODE_NAME)
        types = {"Twist": Twist, "Odometry": Odometry, "Clock": Clock,
                 "JointState": JointState, "TFMessage": TFMessage}
        checks = MotionChecks(node, guard, report, persist, types)
        checks.warmup()
        checks.assert_publishers_isolated()
        checks.stop(0.3)
        report["initial"] = {
            "joint_state": checks.joint_samples[-1],
            "odom_unfiltered": checks.odom_samples["unfiltered"][-1],
            "odom_filtered": checks.odom_samples["filtered"][-1],
            "odom_tf": checks.tf_samples[-1],
            "clock_advances": checks.clock_advances,
        }
        persist()
        duration = args.segment_seconds
        checks.verify_segment("forward", 0.08, 0.0, duration)
        checks.verify_segment("reverse", -0.08, 0.0, duration)
        checks.verify_segment("turn_positive", 0.0, 0.30, duration)
        checks.verify_segment("turn_negative", 0.0, -0.30, duration)
        checks.verify_segment("watchdog_forward", 0.08, 0.0, duration, watchdog=True)
        guard.check()
        report["clock_advances"] = checks.clock_advances
        report["joint_state_samples"] = len(checks.joint_samples)
        report["odom_unfiltered_samples"] = len(checks.odom_samples["unfiltered"])
        report["odom_filtered_samples"] = len(checks.odom_samples["filtered"])
        report["tf_samples"] = len(checks.tf_samples)
        report["pass"] = True
    finally:
        if checks is not None:
            try:
                checks.stop(0.4)
            except Exception as exc:
                report.setdefault("safety", {})["final_zero_error"] = f"{type(exc).__name__}: {exc}"
                report["pass"] = False
            report.setdefault("safety", {})["final_zero_published"] = True
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
    parser.add_argument("--segment-seconds", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.supervisor_pid <= 0 or not math.isfinite(args.segment_seconds) or not 0.5 <= args.segment_seconds <= 5.0:
        parser.error("supervisor-pid must be positive and segment-seconds within [0.5, 5]")
    partition = os.environ.get("IGN_PARTITION", "")
    if os.environ.get("ROS_DOMAIN_ID") != "42" or not partition.startswith("linobot2_g2_") or partition == "linobot2_g2_":
        print("FAIL: require ROS_DOMAIN_ID=42 and a unique linobot2_g2_ partition", file=sys.stderr)
        return 2
    start = time.monotonic()
    report = {"pass": False, "status": "RUNNING", "errors": [],
              "scope": "G2_SIMULATION_ONLY", "world": WORLD, "model": MODEL,
              "supervisor_pid": args.supervisor_pid, "domain_id": "42",
              "partition": partition, "requested_segment_wall_seconds": args.segment_seconds,
              "safety_limits": {"max_linear_x_mps": 0.08, "max_angular_z_rps": 0.30,
                                "segments": 5, "segment_seconds_range": [0.5, 5.0],
                                "hardware_topics_or_devices": "none"}}
    persist = lambda: write_report(args.report, report)
    previous = {}

    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"received signal {signum}")

    try:
        persist()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        run_motion_probe(args, report, persist)
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
