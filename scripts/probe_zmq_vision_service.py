#!/usr/bin/env python3
"""Send one synchronized simulated camera frame to a ``.71``-protocol detector.

Bounded probe: exactly one request, no retries, no robot commands, no Nav2.
The same script targets the local sim detector container (``sim_local``,
Domain 42) or the Jetson ``.71`` (``hardware_71``, Domain 44, separate
approval). Attach to an already running simulation; the probe starts none.
"""

import argparse
import json
import os
from pathlib import Path
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/"linorobot2_vision"/"python"))

from linorobot2_vision.contract import VisionContractError  # noqa: E402
from linorobot2_vision.ros_adapter import capture_from_camera_pair  # noqa: E402
from linorobot2_vision.sim_service_config import validate_zmq_config  # noqa: E402
from linorobot2_vision.zmq_backend import (  # noqa: E402
    VisionZmqBackend, coco80_label_map)

EXPECTED_CAMERA_FRAME_ID = "camera_optical_frame"


def build_report(response, expected_label, elapsed_ms, **extra):
    report = {"status": "PASS", "contract_response": response,
              "client_round_trip_ms": elapsed_ms, **extra}
    if response["status"] != "ok":
        report["status"] = "FAIL"
        report["errors"] = [response["error"]["message"]]
        return report
    detections = response["result"]["detections"]
    report["detection_count"] = len(detections)
    if expected_label:
        report["expected_label"] = expected_label
        if not any(d["label"] == expected_label for d in detections):
            report["status"] = "FAIL"
            report["errors"] = [f"expected detection label {expected_label!r} was absent"]
    return report


def run_probe(args):
    import rclpy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    labels = json.dumps({str(k): v for k, v in coco80_label_map().items()})
    validate_zmq_config(
        args.profile, args.image_endpoint, args.reply_endpoint,
        args.model_version, args.labels_version, labels,
        ros_domain_id=os.environ.get("ROS_DOMAIN_ID"))
    backend = VisionZmqBackend(
        args.image_endpoint, args.reply_endpoint, timeout_sec=args.timeout_sec,
        min_confidence=args.min_confidence, model_name=args.model_name,
        model_version=args.model_version, labels_version=args.labels_version,
        class_id_to_label=coco80_label_map())

    rclpy.init(args=[])
    node = Node("linorobot2_zmq_single_frame_probe",
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    state = {}

    def on_pair(image, camera_info):
        if state:
            return
        try:
            capture = capture_from_camera_pair(
                image, camera_info, clock_domain="ros_sim",
                pair_slop_ns=50_000_000, expected_frame_id=EXPECTED_CAMERA_FRAME_ID)
            started = time.monotonic()
            response = backend.get_response(image, capture)
            state["report"] = build_report(
                response, args.expected_label,
                round((time.monotonic()-started)*1000, 1))
        except (VisionContractError, ValueError) as exc:
            state["report"] = {"status": "FAIL", "errors": [f"{type(exc).__name__}: {exc}"]}

    sync = ApproximateTimeSynchronizer(
        [Subscriber(node, Image, "/camera/image_raw", qos_profile=qos_profile_sensor_data),
         Subscriber(node, CameraInfo, "/camera/camera_info",
                    qos_profile=qos_profile_sensor_data)],
        queue_size=10, slop=0.05, allow_headerless=False)
    sync.registerCallback(on_pair)
    try:
        deadline = time.monotonic()+args.timeout_sec+15.0
        while rclpy.ok() and time.monotonic() < deadline and not state:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not state:
            raise TimeoutError("no synchronized simulated camera frame arrived")
        return {"scope": "ZMQ_SINGLE_SIM_FRAME", "profile": args.profile,
                "image_endpoint": args.image_endpoint,
                "reply_endpoint": args.reply_endpoint,
                "model_version": args.model_version,
                "test_scene": args.test_scene,
                "robot_commands_or_nav2": "none", **state["report"]}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile",
                        choices=["sim_local", "sim_local_isolated", "hardware_71"],
                        default="sim_local")
    parser.add_argument("--image-endpoint", default="tcp://host.docker.internal:5555")
    parser.add_argument("--reply-endpoint", default="tcp://host.docker.internal:5556")
    parser.add_argument("--model-name", default="yolo12n")
    parser.add_argument("--model-version", required=True,
                        help="e.g. weights-sha256:<hash> of the served weights")
    parser.add_argument("--labels-version", default="coco80-ultralytics")
    parser.add_argument("--min-confidence", type=float, default=0.4)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--expected-label", default="")
    parser.add_argument("--test-scene", default="unspecified")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    try:
        report = run_probe(args)
    except Exception as exc:
        report = {"status": "FAIL", "scope": "ZMQ_SINGLE_SIM_FRAME",
                  "errors": [f"{type(exc).__name__}: {exc}"]}
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.report:
        args.report.write_text(text+"\n", encoding="utf-8")
    print(text)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
