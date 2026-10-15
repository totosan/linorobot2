#!/usr/bin/env python3
"""Send one synchronized Fortress camera frame to the real Vision service.

This bounded G6 probe submits exactly one image and does not start Nav2 or
publish robot commands. The image is converted in memory to binary PPM; no
camera image is written to disk.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import urllib.error
import urllib.request
import uuid

WORKSPACE = Path(__file__).resolve().parents[1]
VISION_PYTHON = WORKSPACE/"linorobot2_vision"/"python"
sys.path.insert(0, str(VISION_PYTHON))

from linorobot2_vision.ros_adapter import capture_from_camera_pair  # noqa: E402


DEFAULT_ENDPOINT = "http://192.168.0.150:8082/v1/vision/detection"
EXPECTED_CAMERA_FRAME_ID = "camera_optical_frame"
MAX_UPLOAD_BYTES = 10*1024*1024
MAX_RESPONSE_BYTES = 1024*1024


def image_to_ppm(image):
    """Encode common ROS Image encodings as OpenCV-decodable P6 PPM bytes."""
    width, height = image.width, image.height
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "rgba8": 4,
                            "bgra8": 4, "mono8": 1}
    channels = channels_by_encoding.get(image.encoding)
    if channels is None:
        raise ValueError(f"unsupported camera encoding {image.encoding!r}")
    row_bytes = width*channels
    if width < 1 or height < 1 or image.step < row_bytes:
        raise ValueError("camera image dimensions/step are invalid")
    data = bytes(image.data)
    if len(data) != image.step*height:
        raise ValueError("camera image payload length does not match step*height")

    rgb = bytearray(width*height*3)
    output_offset = 0
    for row_index in range(height):
        row = data[row_index*image.step:row_index*image.step+row_bytes]
        if channels == 1:
            for value in row:
                rgb[output_offset:output_offset+3] = bytes((value, value, value))
                output_offset += 3
        elif image.encoding in ("rgb8", "rgba8"):
            for pixel_offset in range(0, row_bytes, channels):
                rgb[output_offset:output_offset+3] = row[pixel_offset:pixel_offset+3]
                output_offset += 3
        else:
            for pixel_offset in range(0, row_bytes, channels):
                rgb[output_offset:output_offset+3] = row[pixel_offset:pixel_offset+3][::-1]
                output_offset += 3

    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    ppm = header+rgb
    if len(ppm) > MAX_UPLOAD_BYTES:
        raise ValueError(f"encoded image exceeds service upload limit ({MAX_UPLOAD_BYTES} bytes)")
    return ppm


def _multipart(image_bytes, min_confidence):
    boundary = "----linobot2-g6-"+uuid.uuid4().hex
    chunks = [
        f"--{boundary}\r\n".encode("ascii"),
        b'Content-Disposition: form-data; name="image"; filename="fortress-camera.ppm"\r\n',
        b"Content-Type: image/x-portable-pixmap\r\n\r\n",
        image_bytes,
        b"\r\n",
        f"--{boundary}\r\n".encode("ascii"),
        b'Content-Disposition: form-data; name="min_confidence"\r\n\r\n',
        f"{min_confidence:.3f}".encode("ascii"),
        b"\r\n",
        f"--{boundary}--\r\n".encode("ascii"),
    ]
    return boundary, b"".join(chunks)


def _validate_source_response(payload, capture):
    if not isinstance(payload, dict):
        raise ValueError("service response must be a JSON object")
    if type(payload.get("success")) is not bool:
        raise ValueError("service response is missing boolean success")
    if not payload["success"]:
        message = payload.get("message") or payload.get("error") or "service reported failure"
        raise ValueError(f"Vision service failed: {str(message)[:240]}")
    predictions = payload.get("predictions")
    if not isinstance(predictions, list):
        raise ValueError("service response predictions must be an array")

    checked = []
    width, height = capture["width"], capture["height"]
    for index, prediction in enumerate(predictions):
        if not isinstance(prediction, dict):
            raise ValueError(f"prediction {index} must be an object")
        label = prediction.get("label")
        confidence = prediction.get("confidence")
        if not isinstance(label, str) or not label.strip():
            raise ValueError(f"prediction {index} label must be non-empty")
        if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
                or not math.isfinite(float(confidence)) or not 0 <= confidence <= 1):
            raise ValueError(f"prediction {index} confidence must be in [0,1]")
        try:
            box = [int(prediction[key]) for key in ("x_min", "y_min", "x_max", "y_max")]
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"prediction {index} has invalid pixel box") from exc
        x_min, y_min, x_max, y_max = box
        if not (0 <= x_min < x_max <= width and 0 <= y_min < y_max <= height):
            raise ValueError(f"prediction {index} box is outside captured image bounds")
        checked.append({"label": label, "confidence": float(confidence),
                        "bbox_xyxy_px": box})

    for field in ("inferenceMs", "processMs", "analysisRoundTripMs"):
        value = payload.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
            raise ValueError(f"service response {field} must be nonnegative")

    return {
        "model_name": payload.get("moduleId"),
        "inference_device": payload.get("inferenceDevice"),
        "inference_ms": payload["inferenceMs"],
        "process_ms": payload["processMs"],
        "service_round_trip_ms": payload["analysisRoundTripMs"],
        "detections": checked,
    }


def _require_expected_label(report, expected_label):
    if not expected_label:
        return report
    report["expected_label"] = expected_label
    if report.get("status") != "PASS":
        return report
    detections = report.get("detections", [])
    if not any(detection.get("label") == expected_label
               for detection in detections if isinstance(detection, dict)):
        report["status"] = "FAIL"
        report.setdefault("errors", []).append(
            f"expected detection label {expected_label!r} was absent")
    return report


def submit_one_frame(endpoint, image, capture, min_confidence, timeout_sec):
    ppm = image_to_ppm(image)
    boundary, body = _multipart(ppm, min_confidence)
    request = urllib.request.Request(
        endpoint, data=body, method="POST",
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}",
                 "Accept": "application/json"})
    started = time.monotonic()
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec) as response:
            status_code = response.status
            response_bytes = response.read(MAX_RESPONSE_BYTES+1)
    except urllib.error.HTTPError as exc:
        response_bytes = exc.read(MAX_RESPONSE_BYTES+1)
        try:
            payload = json.loads(response_bytes)
        except (UnicodeDecodeError, json.JSONDecodeError):
            payload = {}
        message = payload.get("message") or payload.get("error") or "HTTP request failed"
        raise RuntimeError(f"Vision service HTTP {exc.code}: {str(message)[:240]}") from exc
    elapsed_ms = round((time.monotonic()-started)*1000, 1)
    if len(response_bytes) > MAX_RESPONSE_BYTES:
        raise ValueError("Vision service response exceeded 1 MiB")
    try:
        payload = json.loads(response_bytes)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("Vision service response is not valid JSON") from exc
    result = _validate_source_response(payload, capture)
    result.update({"http_status": status_code,
                   "client_round_trip_ms": elapsed_ms,
                   "uploaded_bytes": len(ppm)})
    return result


class OneFrameProbe:
    def __init__(self, node, endpoint, min_confidence, timeout_sec):
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image

        self.node = node
        self.endpoint = endpoint
        self.min_confidence = min_confidence
        self.timeout_sec = timeout_sec
        self.result = None
        self.error = None
        image_sub = Subscriber(node, Image, "/camera/image_raw",
                               qos_profile=qos_profile_sensor_data)
        info_sub = Subscriber(node, CameraInfo, "/camera/camera_info",
                              qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [image_sub, info_sub], queue_size=10, slop=0.05,
            allow_headerless=False)
        self.sync.registerCallback(self.on_pair)

    def on_pair(self, image, camera_info):
        if self.result is not None or self.error is not None:
            return
        try:
            capture = capture_from_camera_pair(
                image, camera_info, clock_domain="ros_sim", pair_slop_ns=50_000_000,
                expected_frame_id=EXPECTED_CAMERA_FRAME_ID)
            result = submit_one_frame(
                self.endpoint, image, capture, self.min_confidence, self.timeout_sec)
            self.result = {"capture": capture, **result}
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"


def _validate_probe_environment(endpoint, ros_domain_id, partition,
                                attach_to_existing_sim):
    if ros_domain_id != "42":
        raise ValueError("probe is restricted to simulation ROS_DOMAIN_ID=42")
    if (not attach_to_existing_sim
            and not partition.startswith("linorobot2_g6probe_")):
        raise ValueError(
            f"require isolated linorobot2_g6probe_ Ignition partition; got {partition!r}")
    if not endpoint.startswith(("http://", "https://")):
        raise ValueError("endpoint must use http:// or https://")


def run_probe(endpoint, min_confidence, timeout_sec, *,
              attach_to_existing_sim=False, test_scene="unspecified"):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    _validate_probe_environment(
        endpoint, os.environ.get("ROS_DOMAIN_ID"),
        os.environ.get("IGN_PARTITION", ""), attach_to_existing_sim)

    rclpy.init(args=[])
    node = Node("linorobot2_g6_single_frame_probe",
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    probe = OneFrameProbe(node, endpoint, min_confidence, timeout_sec)
    started = time.monotonic()
    try:
        deadline = started+timeout_sec+15.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if probe.result is not None or probe.error is not None:
                break
        if probe.error:
            raise RuntimeError(probe.error)
        if probe.result is None:
            raise TimeoutError("no synchronized simulated camera frame arrived")
        return {"status": "PASS", "scope": "G6_SINGLE_SIM_FRAME_REAL_SERVICE",
                "endpoint": endpoint, "inference": "real Vision service",
                "test_scene": test_scene,
                "simulation_source": ("existing Domain-42 simulation"
                                      if attach_to_existing_sim
                                      else "new headless Fortress instance on Domain 42"),
                "real_camera_or_hardware": "none",
                "robot_commands_or_nav2": "none",
                "elapsed_seconds": round(time.monotonic()-started, 3),
                **probe.result}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _validate_contract_metadata(expected_label, model_version, labels_version,
                                class_id):
    for value, name in ((expected_label, "expected label"),
                         (model_version, "model version"),
                         (labels_version, "labels version")):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be explicitly verified")
    if isinstance(class_id, bool) or not isinstance(class_id, int) or class_id < 0:
        raise ValueError("class ID must be a verified nonnegative integer")


def run_contract_probe(endpoint, min_confidence, timeout_sec, *,
                       model_version, labels_version, class_id,
                       expected_label="person", test_scene="unspecified"):
    """Run one real service request through the ROS Contract-v1 adapter."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    _validate_probe_environment(
        endpoint, os.environ.get("ROS_DOMAIN_ID"),
        os.environ.get("IGN_PARTITION", ""), False)
    _validate_contract_metadata(
        expected_label, model_version, labels_version, class_id)

    from linorobot2_vision.ros_node import VisionAdapterNode

    class ResultMonitor(Node):
        def __init__(self):
            super().__init__("linorobot2_g6_contract_monitor")
            self.result = None
            self.status = None
            self.create_subscription(
                String, "/vision/result", self._on_result, 10)
            self.create_subscription(
                String, "/vision/status", self._on_status, 10)

        def _on_result(self, message):
            self.result = json.loads(message.data)

        def _on_status(self, message):
            self.status = json.loads(message.data)

    parameters = [
        Parameter("backend", value="service"),
        Parameter("use_sim_time", value=True),
        Parameter("clock_domain", value="ros_sim"),
        Parameter("expected_frame_id", value=EXPECTED_CAMERA_FRAME_ID),
        Parameter("service_endpoint_url", value=endpoint),
        Parameter("service_timeout_sec", value=float(timeout_sec)),
        Parameter("service_min_confidence", value=float(min_confidence)),
        Parameter("service_min_interval_sec", value=0.6),
        Parameter("service_max_requests", value=1),
        Parameter("service_max_age_ns", value=1_000_000_000),
        Parameter("service_model_version", value=model_version),
        Parameter("service_labels_version", value=labels_version),
        Parameter("service_label_map_json", value=json.dumps(
            {class_id: expected_label}, separators=(",", ":"))),
    ]
    rclpy.init(args=[])
    adapter = monitor = None
    executor = SingleThreadedExecutor()
    started = time.monotonic()
    try:
        adapter = VisionAdapterNode(parameter_overrides=parameters)
        monitor = ResultMonitor()
        executor.add_node(adapter)
        executor.add_node(monitor)
        deadline = started+timeout_sec+15.0
        while (rclpy.ok() and time.monotonic() < deadline
               and (monitor.result is None or monitor.status is None)):
            executor.spin_once(timeout_sec=0.05)
        if monitor.result is None or monitor.status is None:
            raise TimeoutError("Contract-v1 adapter did not publish result and status")

        contract_result = monitor.result
        detections = []
        if contract_result.get("status") == "ok":
            detections = contract_result.get("result", {}).get("detections", [])
        report = {
            "status": "PASS" if contract_result.get("status") == "ok" else "FAIL",
            "scope": "G6_SINGLE_SIM_FRAME_REAL_SERVICE_CONTRACT_V1",
            "endpoint": endpoint,
            "test_scene": test_scene,
            "simulation_source": "new headless Fortress instance on Domain 42",
            "result_topic": "/vision/result",
            "status_topic": "/vision/status",
            "expected_label": expected_label,
            "service_requests_submitted": adapter._service_requests_submitted,
            "elapsed_seconds": round(time.monotonic()-started, 3),
            "contract_response": contract_result,
            "diagnostic_status": monitor.status,
            "detections": detections,
            "robot_commands_or_nav2": "none",
            "real_camera_or_hardware": "none",
            "errors": [],
        }
        if contract_result.get("status") != "ok":
            report["errors"].append(
                f"Contract-v1 status was {contract_result.get('status')!r}")
        report = _require_expected_label(report, expected_label)
        if adapter._service_requests_submitted != 1:
            report["status"] = "FAIL"
            report["errors"].append(
                "expected exactly one submitted Vision service request")
        return report
    finally:
        if adapter is not None:
            if adapter in executor.get_nodes():
                executor.remove_node(adapter)
            adapter.destroy_node()
        if monitor is not None:
            if monitor in executor.get_nodes():
                executor.remove_node(monitor)
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--min-confidence", type=float, default=0.3)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--attach-to-existing-sim", action="store_true",
        help="use the existing Domain-42 camera stream instead of starting Fortress")
    parser.add_argument(
        "--test-scene", default="unspecified",
        help="name of the configured simulator scene for the evidence report")
    parser.add_argument(
        "--expect-label", default="",
        help="fail the probe unless at least one detection has this exact label")
    parser.add_argument(
        "--through-contract-v1", action="store_true",
        help="use the ROS service adapter and publish Contract v1 to /vision/result")
    parser.add_argument("--model-version", default="")
    parser.add_argument("--labels-version", default="")
    parser.add_argument("--class-id", type=int)
    args = parser.parse_args(argv)
    if not math.isfinite(args.min_confidence) or not 0 <= args.min_confidence <= 1:
        parser.error("min-confidence must be in [0,1]")
    if not math.isfinite(args.timeout) or not 3 <= args.timeout <= 60:
        parser.error("timeout must be 3..60 seconds")
    if args.through_contract_v1:
        try:
            _validate_contract_metadata(
                args.expect_label, args.model_version,
                args.labels_version, args.class_id)
        except ValueError as exc:
            parser.error(str(exc))
    elif args.model_version or args.labels_version or args.class_id is not None:
        parser.error("model/label metadata requires --through-contract-v1")

    report = {"status": "FAIL", "scope": "G6_SINGLE_SIM_FRAME_REAL_SERVICE",
              "real_camera_or_hardware": "none", "errors": []}
    previous_handlers = {}

    def interrupt(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, interrupt)
        if args.through_contract_v1:
            report.update(run_contract_probe(
                args.endpoint, args.min_confidence, args.timeout,
                model_version=args.model_version,
                labels_version=args.labels_version,
                class_id=args.class_id,
                expected_label=args.expect_label,
                test_scene=args.test_scene))
        else:
            report.update(run_probe(
                args.endpoint, args.min_confidence, args.timeout,
                attach_to_existing_sim=args.attach_to_existing_sim,
                test_scene=args.test_scene))
            report = _require_expected_label(report, args.expect_label)
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(f"{report['status']} [{report['scope']}]: {args.report}", flush=True)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
