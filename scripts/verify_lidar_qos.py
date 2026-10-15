#!/usr/bin/env python3
"""Read-only G1 lidar geometry / DDS delivery evidence (not the whole G1 gate).

Run in the sourced Humble environment with ROS_DOMAIN_ID=42 and the existing
Fortress/RViz launch running. No simulator services or command publishers are
used. Geometry helpers and report validation have no ROS dependencies.
"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


MODEL = "linorobot2_2wd"
WORLD = "fortress_test"
BOX_MIN = (1.5, -0.5, 0.0)
BOX_MAX = (2.5, 0.5, 1.0)
ANGLE_MIN = -3.14159
ANGLE_MAX = 3.14159
RANGE_MIN = 0.08
RANGE_MAX = 12.0
RAYS = 360
MIN_SCANS = 10
TOLERANCE = 0.05
TOPICS = ("/clock", "/scan", "/imu/data", "/camera/image_raw",
          "/camera/camera_info")


def finite_vector(values, size, label):
    try:
        result = tuple(float(x) for x in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid {label}") from exc
    if len(result) != size or not all(math.isfinite(x) for x in result):
        raise ValueError(f"invalid {label}: expected {size} finite numbers")
    return result


def unit_quaternion(values):
    """xyzw; normalize slight transport noise, reject missing/invalid rotation."""
    q = finite_vector(values, 4, "quaternion")
    norm = math.sqrt(sum(x * x for x in q))
    if abs(norm - 1.0) > 0.01:
        raise ValueError(f"invalid quaternion norm {norm:g} (expected unit rotation)")
    return tuple(x / norm for x in q)


def quaternion_product(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u,
            w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s)


def rotate(q, vector):
    q = unit_quaternion(q)
    v = finite_vector(vector, 3, "vector")
    result = quaternion_product(quaternion_product(q, (*v, 0.0)),
                                (-q[0], -q[1], -q[2], q[3]))
    return result[:3]


def pose(position, orientation):
    return {"position": finite_vector(position, 3, "position"),
            "orientation": unit_quaternion(orientation)}


def compose(parent, child):
    """Compose world<-model and model<-laser, including roll and pitch."""
    parent = pose(parent["position"], parent["orientation"])
    child = pose(child["position"], child["orientation"])
    offset = rotate(parent["orientation"], child["position"])
    return pose(tuple(a + b for a, b in zip(parent["position"], offset)),
                quaternion_product(parent["orientation"], child["orientation"]))


def pose_delta(start, end):
    start = pose(start["position"], start["orientation"])
    end = pose(end["position"], end["orientation"])
    translation = math.dist(start["position"], end["position"])
    dot = abs(sum(a*b for a, b in zip(start["orientation"], end["orientation"])))
    return {"translation_m": translation,
            "quaternion_angle_rad": 2 * math.acos(min(1.0, dot))}


def require_stable_pose(start, end):
    delta = pose_delta(start, end)
    if delta["translation_m"] > 0.02 or delta["quaternion_angle_rad"] > 0.03:
        raise ValueError(f"world model pose moved during acquisition: {delta}")
    return delta


def parse_model_pose(text):
    """Parse one ign.msgs.Pose_V JSON; protobuf omitted scalars default to zero."""
    try:
        data = json.loads(text)
        entries = data["pose"]
        if not isinstance(entries, list):
            raise ValueError("pose must be a list")
        matches = [p for p in entries if isinstance(p, dict) and p.get("name") == MODEL]
        if len(matches) != 1:
            raise ValueError(f"expected exactly one exact model name {MODEL!r}")
        entry = matches[0]
        position, orientation = entry["position"], entry["orientation"]
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            raise ValueError("position/orientation must be objects")
        return pose([position.get(k, 0.0) for k in "xyz"],
                    [orientation.get(k, 0.0) for k in "xyzw"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Gazebo pose JSON invalid: {exc}") from exc


def read_world_pose(deadline):
    """Bounded observation, never substitute a launch/spawn pose."""
    errors = []
    for suffix in ("dynamic_pose/info", "pose/info"):
        topic = f"/world/{WORLD}/{suffix}"
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        command = ["ign", "topic", "-e", "-t", topic, "-n", "1", "--json-output"]
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=min(5.0, remaining), check=False)
            if result.returncode:
                raise ValueError(f"exit {result.returncode}: {result.stderr.strip()[:400]}")
            return {"topic": topic, "pose": parse_model_pose(result.stdout)}
        except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
            errors.append(f"{topic}: {exc}")
    raise ValueError("no measured Gazebo world model pose (timeout/invalid data): "
                     + "; ".join(errors))


def ray_box(origin, direction):
    """Slab intersection with the actual world-axis-aligned test obstacle."""
    near, far = -math.inf, math.inf
    for o, d, low, high in zip(origin, direction, BOX_MIN, BOX_MAX):
        if abs(d) < 1e-12:
            if o < low or o > high:
                return math.inf
            continue
        a, b = (low - o) / d, (high - o) / d
        near, far = max(near, min(a, b)), min(far, max(a, b))
        if near > far:
            return math.inf
    if far < 0:
        return math.inf
    return near if near >= 0 else far


def predict_rays(world_laser, angle_min, angle_increment, count=RAYS):
    """Nearest 3-D box or ground-plane intersection for each indexed scan ray."""
    sensor = pose(world_laser["position"], world_laser["orientation"])
    origin = sensor["position"]
    if origin[2] <= 0:
        raise ValueError("laser origin is on/below ground plane")
    results = []
    for i in range(count):
        angle = angle_min + i * angle_increment
        direction = rotate(sensor["orientation"], (math.cos(angle), math.sin(angle), 0.0))
        box = ray_box(origin, direction)
        ground = -origin[2] / direction[2] if direction[2] < -1e-12 else math.inf
        distance, target = (box, "box") if box <= ground else (ground, "ground")
        if not math.isfinite(distance) or distance > RANGE_MAX:
            distance, target = math.inf, "none"
        results.append((distance, target))
    return results


def validate_scan_metadata(scan):
    if scan.header.frame_id != "laser":
        raise ValueError(f"scan frame must be 'laser', got {scan.header.frame_id!r}")
    if len(scan.ranges) != RAYS:
        raise ValueError(f"scan must contain exactly {RAYS} rays, got {len(scan.ranges)}")
    for name, expected, tolerance in (("range_min", RANGE_MIN, 1e-5),
                                      ("range_max", RANGE_MAX, 1e-5),
                                      ("angle_min", ANGLE_MIN, 1e-4),
                                      ("angle_max", ANGLE_MAX, 1e-4)):
        value = float(getattr(scan, name))
        if not math.isfinite(value) or abs(value - expected) > tolerance:
            raise ValueError(f"scan {name}={value!r}, expected {expected} +/- {tolerance}")
    increment = float(scan.angle_increment)
    if (not math.isfinite(increment) or increment <= 0 or
            abs(scan.angle_min + (RAYS - 1)*increment - scan.angle_max) > 1e-4):
        raise ValueError("scan angle_increment / endpoint consistency invalid")


def compare_scan(scan, world_laser):
    validate_scan_metadata(scan)
    predictions = predict_rays(world_laser, scan.angle_min, scan.angle_increment)
    box_indices = [i for i, (_, target) in enumerate(predictions) if target == "box"]
    # Endpoints +/-pi are not forward indices and are NOT circular neighbors.
    boundary = {i for i in range(RAYS) for j in (i - 1, i + 1)
                if 0 <= j < RAYS and
                (predictions[i][1] == "box") != (predictions[j][1] == "box")}
    mismatches, boundary_mismatches, invalid, errors = [], [], [], []

    def matches(value, predicted):
        if math.isinf(predicted):
            return value == math.inf or abs(value - RANGE_MAX) <= TOLERANCE
        return math.isfinite(value) and abs(value - predicted) <= TOLERANCE

    for i, (expected, _) in enumerate(predictions):
        measured = float(scan.ranges[i])
        if (math.isnan(measured) or measured == -math.inf or
                (math.isfinite(measured) and
                 not RANGE_MIN - 1e-5 <= measured <= RANGE_MAX + 1e-5)):
            invalid.append(i)
            continue
        if math.isfinite(expected) and math.isfinite(measured):
            errors.append(abs(measured - expected))
        if matches(measured, expected):
            continue
        # Only an adjacent silhouette hit/miss is exempt, not arbitrary bad ranges.
        if i in boundary and any(matches(measured, predictions[j][0])
                                 for j in (i - 1, i + 1) if 0 <= j < RAYS):
            boundary_mismatches.append(i)
        else:
            mismatches.append(i)
    center = min(range(RAYS), key=lambda i: abs(scan.angle_min + i*scan.angle_increment))
    stats = {"rays": RAYS, "frame": scan.header.frame_id, "box_rays": len(box_indices),
             "ground_rays": sum(target == "ground" for _, target in predictions),
             "no_hit_rays": sum(target == "none" for _, target in predictions),
             "finite_measured_rays": sum(math.isfinite(x) for x in scan.ranges),
             "matched_box_rays": sum(matches(float(scan.ranges[i]), predictions[i][0])
                                     for i in box_indices),
             "mismatched_indices": mismatches, "boundary_mismatched_indices": boundary_mismatches,
             "invalid_indices": invalid, "tolerance_m": TOLERANCE,
             "max_error_m": max(errors, default=None), "center_index": center,
             "center_bearing_rad": scan.angle_min + center*scan.angle_increment,
             "box_bearing_min_rad": (scan.angle_min + min(box_indices)*scan.angle_increment
                                     if box_indices else None),
             "box_bearing_max_rad": (scan.angle_min + max(box_indices)*scan.angle_increment
                                     if box_indices else None)}
    stats["pass"] = (len(box_indices) >= 20 and stats["matched_box_rays"] >= 20 and
                     not invalid and not mismatches and len(boundary_mismatches) <= 2)
    return stats


def scan_stamp_ns(scan):
    stamp = scan.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid scan header stamp")
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def require_increasing_stamp(previous, current):
    if previous is not None and current <= previous:
        raise ValueError(f"scan stamps not strictly increasing: {previous} -> {current}")


def summarize_scans(scans):
    return {"compared_scans": len(scans), "required_scans": MIN_SCANS,
            "first_stamp_ns": scans[0]["stamp_ns"] if scans else None,
            "last_stamp_ns": scans[-1]["stamp_ns"] if scans else None,
            "max_error_m": max((s["max_error_m"] for s in scans
                                if s["max_error_m"] is not None), default=None),
            "box_ray_counts": [s["box_rays"] for s in scans],
            "boundary_mismatch_counts": [len(s["boundary_mismatched_indices"]) for s in scans]}


def qos_dict(profile):
    def label(value):
        return getattr(value, "name", str(value))
    return {"reliability": label(profile.reliability), "history": label(profile.history),
            "depth": profile.depth, "durability": label(profile.durability)}


def endpoint_dict(endpoint):
    return {"node_name": endpoint.node_name, "node_namespace": endpoint.node_namespace,
            "endpoint_gid": bytes(endpoint.endpoint_gid).hex(),
            "qos": qos_dict(endpoint.qos_profile)}


def compatibility_evidence(publisher, subscription, checker):
    result, reason = checker(publisher.qos_profile, subscription.qos_profile)
    status = getattr(result, "name", str(result))
    return {"publisher": endpoint_dict(publisher), "subscription": endpoint_dict(subscription),
            "compatibility": status, "reason": reason, "pass": status == "OK"}


def qos_inventory(node, checker, counts):
    """Use discovered endpoints, not assumed bridge or RViz QoS defaults."""
    inventory, failures = {}, []
    for topic in TOPICS:
        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)
        selected = [s for s in subs if s.node_name == node.get_name() and
                    s.node_namespace == node.get_namespace()]
        rviz = [s for s in subs if s.node_name == "rviz2"] if topic == "/scan" else []
        comparisons = [compatibility_evidence(p, s, checker)
                       for p in pubs for s in selected + rviz]
        inventory[topic] = {"publishers": [endpoint_dict(p) for p in pubs],
                            "subscriptions": [endpoint_dict(s) for s in subs],
                            "selected_compatibility": comparisons,
                            "samples": dict(counts[topic])}
        if not pubs:
            failures.append(f"{topic}: no publishers")
        if len(selected) != 2:
            failures.append(f"{topic}: expected two discovered probe subscriptions")
        if topic == "/scan" and not rviz:
            failures.append("/scan: required rviz2 subscription absent")
        for entry in comparisons:
            if not entry["pass"]:
                failures.append(f"{topic}: {entry['subscription']['node_name']} QoS "
                                f"{entry['compatibility']}: {entry['reason']}; contract fix required")
        for profile, samples in counts[topic].items():
            if samples < 1:
                failures.append(f"{topic}: no {profile} subscription sample")
    return inventory, failures


def run_probe(report, deadline):
    # Runtime-only imports keep every mathematical/parser test ROS-free.
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy, qos_check_compatible,
                           qos_profile_sensor_data)
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    node = None
    try:
        rclpy.init(args=[])
        node = Node("linorobot2_g1_lidar_qos_probe", enable_rosout=False,
                    start_parameter_services=False)
        counts = {topic: {"best_effort": 0, "reliable": 0} for topic in TOPICS}
        report["samples"] = counts
        report["qos_profiles"] = {
            "best_effort": qos_dict(qos_profile_sensor_data),
            "reliable": qos_dict(QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE))}
        reliable = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE)
        pending = []
        last_stamp = None
        callback_errors = []
        collecting = False

        def callback(topic, profile):
            def receive(msg):
                nonlocal last_stamp
                counts[topic][profile] += 1
                if topic != "/scan" or profile != "best_effort" or not collecting:
                    return
                try:
                    validate_scan_metadata(msg)
                    stamp = scan_stamp_ns(msg)
                    require_increasing_stamp(last_stamp, stamp)
                    last_stamp = stamp
                    if len(pending) < 100:
                        pending.append(msg)
                    else:
                        raise ValueError("scan TF backlog exceeded 100 samples")
                except ValueError as exc:
                    callback_errors.append(str(exc))
            return receive

        for topic, msg_type in zip(TOPICS, (Clock, LaserScan, Imu, Image, CameraInfo)):
            node.create_subscription(msg_type, topic, callback(topic, "best_effort"),
                                     qos_profile_sensor_data)
            node.create_subscription(msg_type, topic, callback(topic, "reliable"), reliable)
        buffer = Buffer()
        listener = TransformListener(buffer, node, spin_thread=False)
        # Observe before spinning/acquiring scans. Startup/spawn coordinates are not used.
        report["world_pose_start"] = read_world_pose(deadline - 10.0)
        world_model = report["world_pose_start"]["pose"]
        collecting = True
        report["scans"] = []
        last_tf_error = "no scan received"
        next_inventory = 0.0
        while rclpy.ok() and time.monotonic() < deadline - 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if callback_errors:
                raise ValueError("; ".join(callback_errors))
            # Preserve scan ordering when waiting for TF. Zero timeout avoids a
            # sim-clock-blocking tf2 wait; the bounded wall-clock loop spins TF.
            while pending:
                scan = pending[0]
                try:
                    tf = buffer.lookup_transform("base_footprint", "laser",
                                                 Time.from_msg(scan.header.stamp),
                                                 timeout=Duration(seconds=0.0))
                except TransformException as exc:
                    last_tf_error = str(exc)
                    break
                t, q = tf.transform.translation, tf.transform.rotation
                model_laser = pose((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
                world_laser = compose(world_model, model_laser)
                stats = compare_scan(scan, world_laser)
                stats.update({"stamp_ns": scan_stamp_ns(scan),
                              "model_laser_tf": model_laser, "world_laser_pose": world_laser,
                              "tf_lookup": "scan_timestamp; fixed sensor chain; no latest fallback"})
                report["scans"].append(stats)
                pending.pop(0)
                if not stats["pass"]:
                    raise ValueError(f"lidar geometry mismatch at {stats['stamp_ns']}: {stats}")
            if time.monotonic() >= next_inventory:
                inventory, failures = qos_inventory(node, qos_check_compatible, counts)
                report["qos_inventory"] = inventory
                report["qos_errors"] = failures
                next_inventory = time.monotonic() + 0.5
                # Discovery/sample absence may be temporary, incompatible endpoints are not.
                incompatible = [e for e in failures if "contract fix required" in e]
                if incompatible:
                    raise ValueError("; ".join(incompatible))
                if len(report["scans"]) >= MIN_SCANS and not failures:
                    break
        if not rclpy.ok():
            raise ValueError("ROS context stopped during acquisition")
        collecting = False
        report["world_pose_end"] = read_world_pose(deadline)
        report["world_pose_stability"] = require_stable_pose(
            world_model, report["world_pose_end"]["pose"])
        inventory, failures = qos_inventory(node, qos_check_compatible, counts)
        report["qos_inventory"], report["qos_errors"] = inventory, failures
        scans = report["scans"]
        report["scan_stats"] = summarize_scans(scans)
        if len(scans) < MIN_SCANS:
            failures.append(f"timeout: only {len(scans)}/{MIN_SCANS} compared scans; "
                            f"last base_footprint<-laser TF diagnostic: {last_tf_error}")
        if time.monotonic() > deadline:
            failures.append("total wall-clock deadline exceeded")
        if failures:
            raise ValueError("; ".join(failures))
        # Hold the listener alive until every observation has been checked.
        del listener
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="Total wall-clock budget in seconds (default 60, maximum 60)")
    parser.add_argument("--report", type=Path, help="Optional JSON evidence path (no default file)")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 20.0 <= args.timeout <= 60.0:
        parser.error("timeout must be finite and between 20 and 60 seconds")
    start = time.monotonic()
    report = {"pass": False, "errors": [], "world": WORLD, "model": MODEL,
              "domain_id": os.environ.get("ROS_DOMAIN_ID"),
              "scope": "remaining G1 lidar geometry and DDS QoS only; not full G1",
              "metadata": {"rays": RAYS, "angle_min": ANGLE_MIN, "angle_max": ANGLE_MAX,
                           "range_min": RANGE_MIN, "range_max": RANGE_MAX,
                           "box_min": BOX_MIN, "box_max": BOX_MAX, "ground_z": 0.0,
                           "distance_tolerance_m": TOLERANCE, "max_boundary_rays": 2,
                           "minimum_box_rays": 20, "minimum_compared_scans": MIN_SCANS,
                           "pose_translation_tolerance_m": 0.02,
                           "pose_angle_tolerance_rad": 0.03},
              "api_caveats": ["Ignition Transport has no DDS QoS; DDS evidence is ROS-side",
                              "Endpoint compatibility does not prove RViz rendered a scan",
                              "Timestamped fixed-chain TF only; no assumed spawn pose",
                              "No-hit +inf or range_max saturation is accepted"]}
    try:
        if os.environ.get("ROS_DOMAIN_ID") != "42":
            raise ValueError("ROS_DOMAIN_ID must explicitly be 42 before rclpy.init")
        run_probe(report, start + args.timeout)
        report["pass"] = True
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    # Preserve quantitative partial evidence even when acquisition failed early.
    report["scan_stats"] = summarize_scans(report.get("scans", []))
    report["elapsed_seconds"] = time.monotonic() - start
    if args.report:
        try:
            args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        except (OSError, ValueError) as exc:
            report["pass"] = False
            report["errors"].append(f"cannot save JSON report: {exc}")
    if report["pass"]:
        print(f"PASS: {report['scan_stats']['compared_scans']} lidar scans, "
              "five-topic best-effort/reliable delivery and rviz2 QoS checked")
    else:
        print("FAIL: " + "; ".join(report["errors"]))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())