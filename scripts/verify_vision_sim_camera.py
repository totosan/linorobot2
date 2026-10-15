#!/usr/bin/env python3
"""Consume actual Fortress camera/CameraInfo pairs through the Phase-5 mock."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[1]
VISION_PYTHON = WORKSPACE/"linorobot2_vision"/"python"
sys.path.insert(0, str(VISION_PYTHON))

from linorobot2_vision import VisionResultConsumer  # noqa: E402

MAX_PAIR_SKEW_NS = 200_000_000
MAX_RESULT_AGE_NS = 500_000_000


def stamp_ns(stamp):
    sec, nanosec = stamp.sec, stamp.nanosec
    if (isinstance(sec, bool) or not isinstance(sec, int) or sec < 0
            or isinstance(nanosec, bool) or not isinstance(nanosec, int)
            or not 0 <= nanosec < 1_000_000_000):
        raise ValueError("ROS header stamp is invalid")
    return sec*1_000_000_000+nanosec


def validate_camera_pair(image, info, max_skew_ns=MAX_PAIR_SKEW_NS):
    """Validate metadata and sample image bytes without retaining image data."""
    width, height = image.width, image.height
    if (isinstance(width, bool) or not isinstance(width, int) or width <= 0
            or isinstance(height, bool) or not isinstance(height, int) or height <= 0):
        raise ValueError("camera Image dimensions must be positive integers")
    if (info.width, info.height) != (width, height):
        raise ValueError("CameraInfo dimensions do not match Image")
    image_frame = image.header.frame_id.lstrip("/")
    info_frame = info.header.frame_id.lstrip("/")
    if not image_frame or image_frame != info_frame:
        raise ValueError("Image and CameraInfo frame IDs must match and be non-empty")
    if image.encoding != "rgb8":
        raise ValueError(f"camera Image encoding must be rgb8, got {image.encoding!r}")
    minimum_step = width*3
    if isinstance(image.step, bool) or not isinstance(image.step, int) or image.step < minimum_step:
        raise ValueError("camera Image step is shorter than RGB row width")
    data = image.data
    if len(data) != image.step*height:
        raise ValueError("camera Image byte count does not match step*height")
    image_stamp = stamp_ns(image.header.stamp)
    info_stamp = stamp_ns(info.header.stamp)
    if abs(image_stamp-info_stamp) > max_skew_ns:
        raise ValueError("Image and CameraInfo timestamps exceed pairing tolerance")
    if len(info.k) < 9 or not all(math.isfinite(float(value)) for value in info.k[:9]):
        raise ValueError("CameraInfo K matrix is invalid")
    if info.k[0] <= 0 or info.k[4] <= 0:
        raise ValueError("CameraInfo focal lengths must be positive")
    stride = max(1, len(data)//4096)
    samples = {int(data[index]) for index in range(0, len(data), stride)}
    if len(samples) < 2:
        raise ValueError("camera Image samples are uniform or empty")
    return {
        "stamp_ns": image_stamp,
        "clock_domain": "ros_sim",
        "frame_id": image_frame,
        "width": width,
        "height": height,
        "camera_info_stamp_ns": info_stamp,
        "fx": float(info.k[0]),
        "fy": float(info.k[4]),
        "image_encoding": image.encoding,
        "image_bytes": len(data),
        "sampled_byte_values": len(samples),
    }


class EmptyCameraMockBackend:
    """Model-free mock: actual camera frame metadata in, empty result out."""

    def __init__(self, capture):
        self.capture = dict(capture)

    def get_response(self):
        return {
            "schema_version": "1.0",
            "status": "ok",
            "result": {
                "capture": {key: self.capture[key] for key in (
                    "stamp_ns", "clock_domain", "frame_id", "width", "height")},
                "model": {
                    "name": "simulated-camera-empty-mock",
                    "version": "1.0.0",
                    "labels_version": "no-detections-v1",
                },
                "detections": [],
            },
            "error": None,
        }


class SimulatedCameraProbe:
    def __init__(self, rclpy, Image, CameraInfo, Clock, minimum_frames):
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        self.node = Node("linorobot2_phase5_camera_consumer",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.rclpy = rclpy
        self.minimum_frames = minimum_frames
        self.latest_image = None
        self.latest_info = None
        self.clock_ns = None
        self.clock_advances = 0
        self.last_image_stamp = -1
        self.pairs = []
        self.node.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.node.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 10)
        self.node.create_subscription(Clock, "/clock", self.on_clock, 10)

    def on_image(self, msg):
        self.latest_image = msg

    def on_info(self, msg):
        self.latest_info = msg

    def on_clock(self, msg):
        value = stamp_ns(msg.clock)
        if self.clock_ns is not None and value > self.clock_ns:
            self.clock_advances += 1
        if self.clock_ns is not None and value < self.clock_ns:
            raise ValueError("simulation clock rewound during camera consumer test")
        self.clock_ns = value

    def evaluate_pair(self):
        image, info = self.latest_image, self.latest_info
        if image is None or info is None or self.clock_ns is None:
            return
        capture = validate_camera_pair(image, info)
        if capture["stamp_ns"] <= self.last_image_stamp:
            return
        if capture["stamp_ns"] > self.clock_ns:
            return
        consumer = VisionResultConsumer(EmptyCameraMockBackend(capture), MAX_RESULT_AGE_NS)
        outcome = consumer.consume(self.clock_ns, "ros_sim")
        if outcome.state != "empty" or outcome.detections:
            raise ValueError(f"unexpected simulated-camera mock consumer outcome: {outcome}")
        self.last_image_stamp = capture["stamp_ns"]
        self.pairs.append({
            "capture": {key: capture[key] for key in (
                "stamp_ns", "clock_domain", "frame_id", "width", "height")},
            "camera_info_stamp_ns": capture["camera_info_stamp_ns"],
            "fx": capture["fx"],
            "fy": capture["fy"],
            "image_encoding": capture["image_encoding"],
            "image_bytes": capture["image_bytes"],
            "sampled_byte_values": capture["sampled_byte_values"],
            "consumer_state": outcome.state,
            "consumer_age_ns": outcome.age_ns,
            "detections": len(outcome.detections),
        })

    def destroy(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def run_probe(timeout_seconds, minimum_frames):
    import rclpy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image

    if (os.environ.get("ROS_DOMAIN_ID") != "42"
            or not os.environ.get("IGN_PARTITION", "").startswith("linobot2_p5cam_")):
        raise ValueError("require ROS_DOMAIN_ID=42 and an isolated linobot2_p5cam_ partition")
    rclpy.init(args=[])
    probe = SimulatedCameraProbe(rclpy, Image, CameraInfo, Clock, minimum_frames)
    started = time.monotonic()
    deadline = started+timeout_seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(probe.node, timeout_sec=0.05)
            probe.evaluate_pair()
            if len(probe.pairs) >= minimum_frames:
                break
        if len(probe.pairs) < minimum_frames:
            raise TimeoutError(
                f"received {len(probe.pairs)} valid camera/CameraInfo consumer pairs; "
                f"needed {minimum_frames} in {timeout_seconds:g}s")
        if probe.clock_advances < minimum_frames:
            raise ValueError("simulation clock did not advance while consuming camera frames")
        first, last = probe.pairs[0], probe.pairs[-1]
        return {
            "status": "PASS",
            "scope": "PHASE5_SIMULATED_CAMERA_CONSUMER_ONLY",
            "frames_consumed": len(probe.pairs),
            "first_capture_stamp_ns": first["capture"]["stamp_ns"],
            "last_capture_stamp_ns": last["capture"]["stamp_ns"],
            "clock_advances": probe.clock_advances,
            "image_encoding": last["image_encoding"],
            "image_dimensions": [last["capture"]["width"], last["capture"]["height"]],
            "image_frame": last["capture"]["frame_id"],
            "camera_info_focal_px": [last["fx"], last["fy"]],
            "consumer_states": sorted({pair["consumer_state"] for pair in probe.pairs}),
            "detections": 0,
            "real_camera_or_hardware": "none",
            "inference": "none; intentional empty mock backend",
            "elapsed_seconds": round(time.monotonic()-started, 3),
        }
    finally:
        probe.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--minimum-frames", type=int, default=5)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    if (not math.isfinite(args.timeout) or not 5 <= args.timeout <= 60
            or not 1 <= args.minimum_frames <= 30):
        parser.error("timeout must be 5..60 seconds and minimum-frames 1..30")
    report = {"status": "FAIL", "scope": "PHASE5_SIMULATED_CAMERA_CONSUMER_ONLY",
              "errors": [], "real_camera_or_hardware": "none"}
    previous = {}

    def interrupt(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        report.update(run_probe(args.timeout, args.minimum_frames))
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        report["status"] = "FAIL"
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(f"{report['status']} [{report['scope']}]: "
          + (str(args.report) if report["status"] == "PASS" else "; ".join(report["errors"])),
          flush=True)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
