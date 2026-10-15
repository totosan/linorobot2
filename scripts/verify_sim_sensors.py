#!/usr/bin/env python3
"""Check actual Fortress-to-ROS sensor samples for Phase 1 smoke testing."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan


class SensorProbe(Node):
    def __init__(self):
        super().__init__("linorobot2_phase1_sensor_probe")
        self.samples = {}
        self.received_at = {}
        self.counts = {}
        self.clock_ns = None
        self.clock_advances = 0
        self.clock_rewinds = 0
        self.clock_progress_at = time.monotonic()
        self.create_subscription(Clock, "/clock", self.clock_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.create_subscription(Image, "/camera/image_raw", self.image_cb, 10)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.info_cb, 10)

    def clock_cb(self, msg):
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.clock_ns is not None:
            if stamp > self.clock_ns:
                self.clock_advances += 1
                self.clock_progress_at = time.monotonic()
            elif stamp < self.clock_ns:
                self.clock_rewinds += 1
        self.clock_ns = stamp
        self.record("clock", msg.clock)

    def record(self, key, value):
        self.samples[key] = value
        self.received_at[key] = time.monotonic()
        self.counts[key] = self.counts.get(key, 0) + 1

    def scan_cb(self, msg):
        valid = [value for value in msg.ranges if math.isfinite(value)
                 and msg.range_min <= value <= msg.range_max]
        self.record("scan", (len(msg.ranges), len(valid), msg.header.frame_id))

    def imu_cb(self, msg):
        self.record("imu", msg.header.frame_id)

    def image_cb(self, msg):
        data = bytes(msg.data)
        # Sampling across the full byte array avoids storing/printing image payloads.
        if data:
            stride = max(1, len(data) // 4096)
            sampled = data[::stride]
            self.record("image", (msg.width, msg.height, msg.encoding,
                                  len(set(sampled)), len(data), msg.header.frame_id))

    def info_cb(self, msg):
        self.record("camera_info", (msg.width, msg.height, tuple(msg.k),
                        msg.header.frame_id))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=0,
                        help="Additional continuous sensor stability time in seconds")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration must be finite and nonnegative")
    rclpy.init()
    node = SensorProbe()
    required = {"clock", "scan", "imu", "image", "camera_info"}
    deadline = time.monotonic() + 60
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            if required.issubset(node.samples) and node.clock_advances > 0:
                break
        missing = required - node.samples.keys()
        if missing:
            print(f"FAIL: keine ROS-Samples für: {', '.join(sorted(missing))}")
            return 1

        rays, valid_ranges, scan_frame = node.samples["scan"]
        if rays < 180 or valid_ranges < 10 or not scan_frame:
            print(f"FAIL: ungültiger Scan: rays={rays}, valid={valid_ranges}, frame={scan_frame!r}")
            return 1

        width, height, encoding, unique_samples, byte_count, image_frame = node.samples["image"]
        if (width, height) != (640, 480) or unique_samples < 2 or byte_count == 0:
            print("FAIL: Kamera liefert kein plausibles, nicht-konstantes 640x480-Bild: "
                  f"{width}x{height} {encoding}, sampled_colors={unique_samples}")
            return 1

        info_width, info_height, intrinsics, info_frame = node.samples["camera_info"]
        if ((info_width, info_height) != (width, height) or intrinsics[0] <= 0
            or intrinsics[4] <= 0 or not image_frame or image_frame != info_frame):
            print("FAIL: CameraInfo-Auflösung/Intrinsics passen nicht zum Bild.")
            return 1

        if not node.samples["imu"]:
            print("FAIL: IMU-Frame fehlt.")
            return 1

        if node.clock_advances == 0 or node.clock_rewinds:
            print("FAIL: Simulationszeit steht still oder springt rückwärts.")
            return 1

        print("PASS: ROS clock, LaserScan, IMU, Bild und CameraInfo empfangen.")
        print(f"scan: {rays} rays, {valid_ranges} gültige Messwerte, frame={scan_frame}")
        print(f"camera: {width}x{height} {encoding}, {unique_samples} verschiedene Samples, "
              f"frame={image_frame}")
        print(f"camera_info: {info_width}x{info_height}, fx={intrinsics[0]:.3f}, fy={intrinsics[4]:.3f}")
        if args.duration:
            start = time.monotonic()
            deadline = start + args.duration
            report_at = start
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.25)
                now = time.monotonic()
                stale = [key for key in required if now - node.received_at[key] > 10]
                if stale or now - node.clock_progress_at > 10 or node.clock_rewinds:
                    print(f"FAIL: Daten-/Clock-Stillstand: stale={sorted(stale)}, "
                          f"clock_rewinds={node.clock_rewinds}", flush=True)
                    return 1
                if now >= report_at:
                    print(f"STABIL: {now - start:.0f}/{args.duration:g}s, "
                          f"scan={node.counts['scan']}, image={node.counts['image']}, "
                          f"clock_advances={node.clock_advances}", flush=True)
                    report_at = now + 20
            if not rclpy.ok():
                print("FAIL: ROS-Kontext während Stabilitätstest beendet.", flush=True)
                return 1
            print(f"PASS: {args.duration:g}s kontinuierlicher Sensor-/Clock-Datenfluss.", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
