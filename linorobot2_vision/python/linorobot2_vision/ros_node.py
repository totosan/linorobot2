"""ROS 2 transport adapter for the transport-neutral vision contract."""

import json
import math
from pathlib import Path
import queue
from concurrent.futures import ThreadPoolExecutor
import time

from ament_index_python.packages import get_package_share_directory
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .consumer import ConsumerOutcome, VisionOffBackend, VisionResultConsumer
from .contract import FixtureVisionBackend, VisionContractError
from .ros_adapter import (
    bind_fixture_capture,
    capture_from_camera_pair,
    error_response,
    parse_label_map,
    response_for_outcome,
    status_payload,
)
from .service_backend import VisionHTTPServiceBackend
from .zmq_backend import VisionZmqBackend


class _FixedResponseBackend:
    def __init__(self, response):
        self.response = response

    def get_response(self):
        return self.response


class VisionAdapterNode(Node):
    """Pair camera metadata, validate mock/off results, and publish JSON topics."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__("linorobot2_vision_adapter",
                         parameter_overrides=parameter_overrides)
        self.declare_parameter("backend", "off")
        self.declare_parameter("fixture_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("result_topic", "/vision/result")
        self.declare_parameter("status_topic", "/vision/status")
        self.declare_parameter("clock_domain", "ros_sim")
        self.declare_parameter("max_age_ns", 250_000_000)
        self.declare_parameter("pair_slop_ns", 50_000_000)
        self.declare_parameter("expected_frame_id", "")
        self.declare_parameter("label_map_json", '{"1":"test_object"}')
        self.declare_parameter(
            "service_endpoint_url",
            "http://192.168.0.150:8082/v1/vision/detection")
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("service_min_confidence", 0.3)
        self.declare_parameter("service_min_interval_sec", 1.0)
        self.declare_parameter("service_max_requests", 0)
        self.declare_parameter("service_max_age_ns", 1_000_000_000)
        self.declare_parameter("service_model_version", "")
        self.declare_parameter("service_labels_version", "")
        self.declare_parameter("service_label_map_json", "{}")
        self.declare_parameter("zmq_image_endpoint", "tcp://127.0.0.1:5555")
        self.declare_parameter("zmq_reply_endpoint", "tcp://127.0.0.1:5556")
        self.declare_parameter("zmq_model_name", "yolo12n")
        self.declare_parameter("zmq_model_version", "")
        self.declare_parameter("zmq_labels_version", "")
        self.declare_parameter("zmq_label_map_json", "{}")
        self.declare_parameter("zmq_timeout_sec", 60.0)

        self.backend_mode = self.get_parameter("backend").value
        if self.backend_mode not in ("off", "mock", "service", "zmq"):
            raise ValueError("backend must be 'off', 'mock', 'service' or 'zmq'")
        # Both remote backends share the throttled, single-flight worker path.
        self.remote_backend = self.backend_mode in ("service", "zmq")

        self.clock_domain = self.get_parameter("clock_domain").value
        if self.clock_domain not in ("ros_sim", "unix_utc"):
            raise ValueError("clock_domain must be 'ros_sim' or 'unix_utc'")
        self.max_age_ns = self.get_parameter("max_age_ns").value
        self.pair_slop_ns = self.get_parameter("pair_slop_ns").value
        if (isinstance(self.max_age_ns, bool) or not isinstance(self.max_age_ns, int)
                or self.max_age_ns < 0):
            raise ValueError("max_age_ns must be a nonnegative integer")
        if (isinstance(self.pair_slop_ns, bool) or not isinstance(self.pair_slop_ns, int)
                or self.pair_slop_ns < 0):
            raise ValueError("pair_slop_ns must be a nonnegative integer")
        self.expected_frame_id = self.get_parameter("expected_frame_id").value
        if not isinstance(self.expected_frame_id, str):
            raise ValueError("expected_frame_id must be a string")
        self.class_id_to_label = parse_label_map(
            self.get_parameter("label_map_json").value)
        self.service_min_interval_sec = self.get_parameter(
            "service_min_interval_sec").value
        if (isinstance(self.service_min_interval_sec, bool)
                or not isinstance(self.service_min_interval_sec, (int, float))
                or not math.isfinite(float(self.service_min_interval_sec))
                or not 0.6 <= self.service_min_interval_sec <= 60):
            raise ValueError("service_min_interval_sec must be in [0.6, 60]")
        self.service_max_requests = self.get_parameter(
            "service_max_requests").value
        if (isinstance(self.service_max_requests, bool)
                or not isinstance(self.service_max_requests, int)
                or self.service_max_requests < 0):
            raise ValueError("service_max_requests must be a nonnegative integer")
        self._service_requests_submitted = 0
        self._last_service_request_at = float("-inf")
        self._service_future = None
        self._service_executor = None
        self._service_results = queue.Queue()
        self._service_result_timer = None

        self.backend = self._create_backend()
        if self.remote_backend:
            service_max_age_ns = self.get_parameter("service_max_age_ns").value
            if (isinstance(service_max_age_ns, bool)
                    or not isinstance(service_max_age_ns, int)
                    or service_max_age_ns < 0):
                raise ValueError("service_max_age_ns must be a nonnegative integer")
            self._service_executor = ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="vision-http")
            self._service_result_timer = self.create_timer(
                0.05, self._drain_service_result)
        self.result_publisher = self.create_publisher(
            String, self.get_parameter("result_topic").value, 10)
        self.status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)

        image_subscriber = Subscriber(
            self, Image, self.get_parameter("image_topic").value,
            qos_profile=qos_profile_sensor_data)
        camera_info_subscriber = Subscriber(
            self, CameraInfo, self.get_parameter("camera_info_topic").value,
            qos_profile=qos_profile_sensor_data)
        self.synchronizer = ApproximateTimeSynchronizer(
            [image_subscriber, camera_info_subscriber],
            queue_size=10,
            slop=self.pair_slop_ns/1_000_000_000.0,
            allow_headerless=False)
        self.synchronizer.registerCallback(self._on_camera_pair)

        self.get_logger().info(
            f"vision adapter ready: backend={self.backend_mode}, "
            f"clock_domain={self.clock_domain}; no robot command interface")

    def _create_backend(self):
        if self.backend_mode == "off":
            return VisionOffBackend()
        if self.backend_mode == "zmq":
            try:
                import zmq  # noqa: F401
            except ImportError as exc:
                raise ValueError("backend 'zmq' requires pyzmq") from exc
            return VisionZmqBackend(
                self.get_parameter("zmq_image_endpoint").value,
                self.get_parameter("zmq_reply_endpoint").value,
                timeout_sec=self.get_parameter("zmq_timeout_sec").value,
                min_confidence=self.get_parameter("service_min_confidence").value,
                model_name=self.get_parameter("zmq_model_name").value,
                model_version=self.get_parameter("zmq_model_version").value,
                labels_version=self.get_parameter("zmq_labels_version").value,
                class_id_to_label=parse_label_map(
                    self.get_parameter("zmq_label_map_json").value))
        if self.backend_mode == "service":
            service_label_map = parse_label_map(
                self.get_parameter("service_label_map_json").value,
                allow_empty=True)
            return VisionHTTPServiceBackend(
                self.get_parameter("service_endpoint_url").value,
                timeout_sec=self.get_parameter("service_timeout_sec").value,
                min_confidence=self.get_parameter("service_min_confidence").value,
                model_version=self.get_parameter("service_model_version").value,
                labels_version=self.get_parameter("service_labels_version").value,
                class_id_to_label=service_label_map)
        fixture_path = self.get_parameter("fixture_path").value
        if not fixture_path:
            fixture_path = str(Path(get_package_share_directory("linorobot2_vision"))
                               / "fixtures" / "empty.json")
        if not Path(fixture_path).is_absolute():
            raise ValueError("fixture_path must be an absolute path when specified")
        return FixtureVisionBackend(fixture_path)

    def _publish(self, response, outcome, capture=None):
        result_message = String()
        result_message.data = json.dumps(response, separators=(",", ":"), sort_keys=True)
        self.result_publisher.publish(result_message)

        status_message = String()
        status_message.data = json.dumps(
            status_payload(outcome, capture), separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(status_message)

    def _publish_contract_error(self, code, message, capture=None):
        response = error_response(code, message)
        outcome = ConsumerOutcome("error", error_code=code, message=str(message))
        self._publish(response, outcome, capture)

    def _drain_service_result(self):
        try:
            capture, response, error = self._service_results.get_nowait()
        except queue.Empty:
            return
        if error is not None:
            code = ("invalid_service_response" if isinstance(error, VisionContractError)
                    else "service_request_failed")
            self._publish_contract_error(
                code, f"{type(error).__name__}: {error}", capture)
            return
        outcome = VisionResultConsumer(
            _FixedResponseBackend(response),
            self.get_parameter("service_max_age_ns").value).consume(
                self.get_clock().now().nanoseconds, self.clock_domain)
        self._publish(response_for_outcome(response, outcome), outcome, capture)

    def _on_camera_pair(self, image, camera_info):
        try:
            capture = capture_from_camera_pair(
                image, camera_info,
                clock_domain=self.clock_domain,
                pair_slop_ns=self.pair_slop_ns,
                expected_frame_id=self.expected_frame_id)
        except VisionContractError as exc:
            self._publish_contract_error("invalid_capture_metadata", str(exc))
            return

        try:
            if self.remote_backend:
                if (self.service_max_requests > 0
                        and self._service_requests_submitted >= self.service_max_requests):
                    return
                now_monotonic = time.monotonic()
                if (now_monotonic-self._last_service_request_at
                        < self.service_min_interval_sec):
                    return
                if (self._service_future is not None
                        and not self._service_future.done()):
                    return
                self._last_service_request_at = now_monotonic
                future = self._service_executor.submit(
                    self.backend.get_response, image, capture)
                self._service_requests_submitted += 1
                self._service_future = future

                def on_service_done(completed, capture_snapshot=dict(capture)):
                    try:
                        self._service_results.put(
                            (capture_snapshot, completed.result(), None))
                    except Exception as exc:
                        self._service_results.put((capture_snapshot, None, exc))

                future.add_done_callback(on_service_done)
                return
            else:
                response = self.backend.get_response()
            if self.backend_mode == "mock":
                response = bind_fixture_capture(
                    response, capture, self.class_id_to_label)
        except VisionContractError as exc:
            code = ("invalid_service_response" if self.remote_backend
                    else "invalid_fixture_response")
            self._publish_contract_error(code, str(exc), capture)
            return
        except Exception as exc:
            self._publish_contract_error(
                "backend_exception", f"{type(exc).__name__}: {exc}", capture)
            return

        outcome = VisionResultConsumer(
            _FixedResponseBackend(response), self.max_age_ns).consume(
                self.get_clock().now().nanoseconds, self.clock_domain)
        self._publish(response_for_outcome(response, outcome), outcome, capture)

    def destroy_node(self):
        if self._service_executor is not None:
            self._service_executor.shutdown(wait=True, cancel_futures=True)
            self._service_executor = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionAdapterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
