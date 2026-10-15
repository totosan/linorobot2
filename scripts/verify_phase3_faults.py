#!/usr/bin/env python3
"""Verify camera-topic loss/recovery after a clean Fortress relaunch (SIL only)."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from verify_phase3 import assert_tf_ownership
from verify_sim_motion import MODEL, WORLD, read_model_pose, write_report

NODE_NAME = "linobot2_g3_fault_probe"
CAMERA_SERVICE = "/g3_fault_injector/set_camera_enabled"
SENSOR_KEYS = ("scan", "imu", "image", "camera_info")


class FaultChecks:
    def __init__(self, node, types):
        self.node = node
        self.counts = {key: 0 for key in ("clock", *SENSOR_KEYS, "joint", "odom_raw", "odom", "tf", "command")}
        self.received_at = {}
        self.clock_ns = None
        self.clock_advances = 0
        self.clock_progress_at = time.monotonic()
        self.nonzero_commands = 0
        self.error = None
        self.command_pub = node.create_publisher(types["Twist"], "/cmd_vel", 10)
        node.create_subscription(types["Clock"], "/clock", lambda msg: self.record_clock(msg), 10)
        node.create_subscription(types["LaserScan"], "/scan", lambda msg: self.record("scan", msg), 10)
        node.create_subscription(types["Imu"], "/imu/data", lambda msg: self.record("imu", msg), 10)
        node.create_subscription(types["Image"], "/camera/image_raw", lambda msg: self.record("image", msg), 10)
        node.create_subscription(types["CameraInfo"], "/camera/camera_info",
                                 lambda msg: self.record("camera_info", msg), 10)
        node.create_subscription(types["JointState"], "/joint_states", lambda msg: self.record("joint", msg), 10)
        node.create_subscription(types["Odometry"], "/odom/unfiltered", lambda msg: self.record("odom_raw", msg), 10)
        node.create_subscription(types["Odometry"], "/odom", lambda msg: self.record("odom", msg), 10)
        node.create_subscription(types["TFMessage"], "/tf", lambda msg: self.record("tf", msg), 10)
        node.create_subscription(types["Twist"], "/cmd_vel", self.record_command, 10)

    def record(self, key, _message):
        self.counts[key] += 1
        self.received_at[key] = time.monotonic()

    def record_clock(self, msg):
        stamp = msg.clock.sec*1_000_000_000+msg.clock.nanosec
        now = time.monotonic()
        if self.clock_ns is not None:
            if stamp < self.clock_ns:
                self.error = self.error or "simulation clock rewound after clean restart"
            elif stamp > self.clock_ns:
                self.clock_advances += 1
                self.clock_progress_at = now
        self.clock_ns = stamp
        self.record("clock", msg)

    def record_command(self, msg):
        if any(abs(value) > 1e-9 for value in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self.nonzero_commands += 1
        self.record("command", msg)

    def spin(self, duration):
        import rclpy
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, deadline-time.monotonic())))
            if self.error:
                raise RuntimeError(self.error)
            if self.clock_ns is not None and time.monotonic()-self.clock_progress_at > 2.0:
                raise TimeoutError("simulation clock stopped during fault/restart verification")

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and not predicate():
            self.spin(min(0.05, max(0.0, deadline-time.monotonic())))
        if not predicate():
            raise TimeoutError(f"timed out waiting for {description}")


def set_camera_enabled(node, enabled):
    import rclpy
    from std_srvs.srv import SetBool
    client = node.create_client(SetBool, CAMERA_SERVICE)
    if not client.wait_for_service(timeout_sec=5.0):
        raise TimeoutError(f"camera fault injection service unavailable: {CAMERA_SERVICE}")
    future = client.call_async(SetBool.Request(data=enabled))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None or not future.result().success:
        raise RuntimeError(f"camera topic fault toggle failed (enabled={enabled})")
    return future.result().message


def publisher_names(node, topic):
    return {endpoint.node_name.lstrip("/")
            for endpoint in node.get_publishers_info_by_topic(topic)}


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def run(args, report):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
    from tf2_msgs.msg import TFMessage
    from verify_camera_gui import OwnershipGuard

    if alive(args.previous_supervisor_pid):
        raise RuntimeError("previous supervised simulator launch is still alive")
    guard = OwnershipGuard(args.supervisor_pid, partition_prefix="linobot2_g3_")
    guard.check()
    node = rclpy.create_node(NODE_NAME)
    camera_disabled = False
    try:
        types = {"Twist": Twist, "Clock": Clock, "LaserScan": LaserScan, "Imu": Imu,
                 "Image": Image, "CameraInfo": CameraInfo, "JointState": JointState,
                 "Odometry": Odometry, "TFMessage": TFMessage}
        checks = FaultChecks(node, types)
        checks.wait_for(lambda: checks.clock_advances >= 2
                        and all(checks.counts[key] >= 3 for key in (*SENSOR_KEYS, "joint", "odom_raw", "odom", "tf")),
                        25.0, "fresh clock, sensors, odometry and TF after restart")
        guard.check()
        assert_tf_ownership(node, report)

        cmd_publishers = publisher_names(node, "/cmd_vel")
        expected_cmd_publishers = {NODE_NAME, "command_timeout"}
        if cmd_publishers != expected_cmd_publishers:
            raise RuntimeError(f"unexpected /cmd_vel publishers after restart: {sorted(cmd_publishers)}")
        image_publishers = publisher_names(node, "/camera/image_raw")
        if image_publishers != {"g3_camera_fault_proxy"}:
            raise RuntimeError(f"camera image topic is not isolated behind the fault proxy: {sorted(image_publishers)}")
        report["restart"] = {
            "previous_supervisor_pid": args.previous_supervisor_pid,
            "new_supervisor_pid": args.supervisor_pid,
            "previous_partition": args.previous_partition,
            "new_partition": os.environ["IGN_PARTITION"],
            "old_supervisor_exited": True,
            "new_world_ready": True,
            "fresh_clock_sensor_odom_tf_samples": True,
            "initial_clock_advances": checks.clock_advances,
            "initial_samples": dict(checks.counts),
            "unique_odom_tf_owner": report["odom_tf_owner"],
        }
        if args.previous_partition == os.environ["IGN_PARTITION"]:
            raise RuntimeError("simulator restart reused the old Ignition partition")

        start_pose = read_model_pose(time.monotonic()+7)
        before = dict(checks.counts)
        service_down = set_camera_enabled(node, False)
        camera_disabled = True
        report["sensor_dropout"] = {"topic": "/camera/image_raw",
                                    "injection": "ROS bridge output gated; Ignition camera remains active",
                                    "service_disable": service_down}
        checks.wait_for(lambda: time.monotonic()-checks.received_at.get("image", time.monotonic()) >= 0.75,
                        4.0, "camera image stream to become stale")
        quiet_count = checks.counts["image"]
        dropout_begin = time.monotonic()
        checks.spin(1.0)
        dropout_duration = time.monotonic()-dropout_begin
        if checks.counts["image"] != quiet_count:
            raise RuntimeError("camera images continued after injected topic outage")
        flowing = {key: checks.counts[key]-before[key] for key in ("clock", "scan", "imu", "camera_info", "joint", "odom_raw", "odom", "tf")}
        minimums = {"clock": 5, "scan": 2, "imu": 3, "camera_info": 2,
                    "joint": 2, "odom_raw": 2, "odom": 2, "tf": 2}
        insufficient = {key: count for key, count in flowing.items() if count < minimums[key]}
        if insufficient:
            raise RuntimeError(f"non-camera streams did not stay healthy during outage: {insufficient}")
        if checks.nonzero_commands:
            raise RuntimeError(f"nonzero command appeared after restart during stationary sensor test: {checks.nonzero_commands}")
        outage_pose = read_model_pose(time.monotonic()+7)
        outage_motion = math.dist(start_pose["position"][:2], outage_pose["position"][:2])
        if outage_motion > 0.02:
            raise RuntimeError(f"robot moved during stationary sensor outage: {outage_motion:.4f}m")

        service_up = set_camera_enabled(node, True)
        camera_disabled = False
        recovery_start = checks.counts["image"]
        checks.wait_for(lambda: checks.counts["image"] >= recovery_start+3,
                        8.0, "camera image samples to recover")
        guard.check()
        final_pose = read_model_pose(time.monotonic()+7)
        total_motion = math.dist(start_pose["position"][:2], final_pose["position"][:2])
        if checks.nonzero_commands or total_motion > 0.02:
            raise RuntimeError("stale command or motion detected after restart/camera recovery")
        report["sensor_dropout"].update({
            "images_before": before["image"], "images_during_outage": quiet_count,
            "image_samples_after_reenable": checks.counts["image"]-recovery_start,
            "outage_duration_wall_seconds": dropout_duration,
            "other_topic_samples_during_outage": flowing,
            "recovery_service": service_up,
            "nonzero_command_echoes_after_restart": checks.nonzero_commands,
            "stationary_motion_m": total_motion, "pass": True,
        })
        report["restart"].update({"clock_advances_after_restart": checks.clock_advances,
                                   "stale_nonzero_commands_after_restart": checks.nonzero_commands,
                                   "motion_without_new_command_m": total_motion,
                                   "pass": True})
        report["sensor_failure_tested"] = True
        report["simulation_restart_tested"] = True
        report["pass"] = True
    finally:
        if camera_disabled and rclpy.ok():
            try:
                set_camera_enabled(node, True)
                report["cleanup_camera_forwarding_restored"] = True
            except Exception as exc:
                report["errors"].append(f"camera proxy cleanup failed: {type(exc).__name__}: {exc}")
        node.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--supervisor-pid", type=int, required=True)
    parser.add_argument("--previous-supervisor-pid", type=int, required=True)
    parser.add_argument("--previous-partition", required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--core-report", type=Path, required=True)
    args = parser.parse_args(argv)
    partition = os.environ.get("IGN_PARTITION", "")
    if (args.supervisor_pid <= 0 or args.previous_supervisor_pid <= 0
            or args.supervisor_pid == args.previous_supervisor_pid
            or os.environ.get("ROS_DOMAIN_ID") != "42"
            or not partition.startswith("linobot2_g3_") or partition == "linobot2_g3_"
            or not args.previous_partition.startswith("linobot2_g3_")
            or args.previous_partition == partition):
        parser.error("require distinct supervised generations in unique G3 partitions and ROS_DOMAIN_ID=42")
    report = {"pass": False, "status": "RUNNING", "errors": [],
              "scope": "G3_SIMULATION_ONLY", "world": WORLD, "model": MODEL,
              "domain_id": "42", "partition": partition,
              "simulation_restart_tested": False, "sensor_failure_tested": False}
    started = time.monotonic()
    persist = lambda: write_report(args.report, report)
    previous = {}

    def interrupt(signum, _frame):
        raise KeyboardInterrupt(f"received signal {signum}")

    try:
        import rclpy
        rclpy.init(args=[])
        persist()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        run(args, report)
        if report["pass"]:
            core = json.loads(args.core_report.read_text())
            if not core.get("pass"):
                raise RuntimeError("core G3 report was not PASS; refusing to merge fault acceptance")
            core["sensor_failure_tested"] = True
            core["simulation_restart_tested"] = True
            core["fault_injection_restart"] = {
                "pass": True, "report": str(args.report),
                "simulator_restart": report["restart"],
                "sensor_dropout": report["sensor_dropout"],
            }
            write_report(args.core_report, core)
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        report["pass"] = False
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except ImportError:
            pass
        report["elapsed_seconds"] = time.monotonic()-started
        report["status"] = "PASS" if report["pass"] else "FAIL"
        persist()
    print(f"{report['status']} [{report['scope']}]: "
          + (str(args.report) if report["pass"] else "; ".join(report["errors"])), flush=True)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
