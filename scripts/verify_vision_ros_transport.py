#!/usr/bin/env python3
"""Verify live off/mock Image+CameraInfo to Vision-contract ROS transport."""

import argparse
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"linorobot2_vision/python"))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from linorobot2_vision import validate_response


class VisionTransportProbe(Node):
    def __init__(self, mode):
        super().__init__("vision_ros_transport_probe")
        self.mode = mode
        self.results = []
        self.statuses = []
        self.errors = []
        self.create_subscription(String, "/vision/result", self._on_result, 10)
        self.create_subscription(String, "/vision/status", self._on_status, 10)

    def _on_result(self, message):
        try:
            response = validate_response(json.loads(message.data))
            if self.mode == "off":
                if (response["status"] != "unavailable"
                        or response["error"]["code"] != "vision_disabled"):
                    raise ValueError("off mode did not report vision_disabled")
            else:
                if response["status"] != "ok":
                    raise ValueError("mock mode did not publish an ok contract result")
                if not response["result"]["detections"]:
                    raise ValueError("mock mode fixture has no detections")
                capture = response["result"]["capture"]
                if capture["clock_domain"] != "ros_sim":
                    raise ValueError("mock capture is not tagged with ros_sim time")
            self.results.append(response)
        except Exception as exc:
            self.errors.append(f"result: {type(exc).__name__}: {exc}")

    def _on_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict) or not isinstance(status.get("state"), str):
                raise ValueError("status payload lacks a state")
            expected = "unavailable" if self.mode == "off" else "ok"
            if status["state"] != expected:
                raise ValueError(f"expected status {expected!r}, got {status['state']!r}")
            if status.get("detection_count") != (0 if self.mode == "off" else 1):
                raise ValueError("status detection_count does not match selected backend")
            if self.mode == "mock" and not isinstance(status.get("age_ns"), int):
                raise ValueError("mock status lacks validated capture age")
            self.statuses.append(status)
        except Exception as exc:
            self.errors.append(f"status: {type(exc).__name__}: {exc}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("off", "mock"), required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--min-pairs", type=int, default=3)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--domain", type=str, default="42")
    args = parser.parse_args(argv)
    report = {
        "pass": False,
        "scope": "G5_VISION_ROS_TRANSPORT_SIL_ONLY",
        "mode": args.mode,
        "expected_state": "unavailable" if args.mode == "off" else "ok",
        "expected_result_count": 0 if args.mode == "off" else 1,
        "domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "required_domain": args.domain,
        "errors": [],
        "hardware_topics_or_devices": "none",
        "command_topics_published": [],
    }
    if args.timeout <= 0 or args.min_pairs < 1:
        parser.error("timeout must be positive and min-pairs must be >= 1")
    if report["domain_id"] != args.domain:
        report["errors"].append(
            f"transport probe requires ROS_DOMAIN_ID={args.domain}")
        return _write_report(args.report, report)

    rclpy.init(args=None)
    probe = VisionTransportProbe(args.mode)
    try:
        deadline = time.monotonic()+args.timeout
        while (len(probe.results) < args.min_pairs
               or len(probe.statuses) < args.min_pairs):
            if probe.errors:
                raise RuntimeError(probe.errors[0])
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"received {len(probe.results)} result and "
                    f"{len(probe.statuses)} status messages; "
                    f"required {args.min_pairs}")
            rclpy.spin_once(probe, timeout_sec=0.05)

        publisher_names = {
            item.node_name.strip().lstrip("/")
            for item in probe.get_publishers_info_by_topic("/cmd_vel")}
        if "linorobot2_vision_adapter" in publisher_names:
            raise RuntimeError("vision adapter unexpectedly publishes /cmd_vel")
        report.update({
            "result_messages": len(probe.results),
            "status_messages": len(probe.statuses),
            "first_capture": (probe.results[0].get("result") or {}).get("capture"),
            "last_capture": (probe.results[-1].get("result") or {}).get("capture"),
            "detection_count": len((probe.results[-1].get("result") or {})
                                    .get("detections", [])),
            "cmd_vel_publishers_observed": sorted(publisher_names),
            "pass": True,
        })
    except Exception as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
    return _write_report(args.report, report)


def _write_report(path, report):
    report["status"] = "PASS" if report["pass"] else "FAIL"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(f"{report['status']} [G5_VISION_ROS_TRANSPORT_SIL_ONLY]: {path}", flush=True)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
