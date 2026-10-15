"""ROS transport smoke tests for the Vision adapter's off/mock modes."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
from threading import Thread
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/"linorobot2_vision"
sys.path.insert(0, str(PACKAGE/"python"))

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from linorobot2_vision.ros_node import VisionAdapterNode
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS 2 Humble Python packages are required")
class VisionAdapterRosNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def _exercise_backend(self, backend):
        suffix = f"/vision_adapter_test/{backend}"
        result_messages = []
        status_messages = []
        parameters = [
            Parameter("backend", value=backend),
            Parameter("use_sim_time", value=True),
            Parameter("clock_domain", value="ros_sim"),
            Parameter("image_topic", value=suffix+"/image"),
            Parameter("camera_info_topic", value=suffix+"/camera_info"),
            Parameter("result_topic", value=suffix+"/result"),
            Parameter("status_topic", value=suffix+"/status"),
        ]
        adapter = VisionAdapterNode(parameter_overrides=parameters)
        probe = Node(f"vision_adapter_probe_{backend}")
        image_publisher = probe.create_publisher(
            Image, suffix+"/image", qos_profile_sensor_data)
        info_publisher = probe.create_publisher(
            CameraInfo, suffix+"/camera_info", qos_profile_sensor_data)
        probe.create_subscription(String, suffix+"/result",
                                  lambda msg: result_messages.append(msg.data), 10)
        probe.create_subscription(String, suffix+"/status",
                                  lambda msg: status_messages.append(msg.data), 10)
        executor = SingleThreadedExecutor()
        executor.add_node(adapter)
        executor.add_node(probe)
        try:
            discovery_deadline = time.monotonic()+3.0
            while (image_publisher.get_subscription_count() == 0
                   or info_publisher.get_subscription_count() == 0):
                if time.monotonic() >= discovery_deadline:
                    self.fail("adapter camera subscriptions were not discovered")
                executor.spin_once(timeout_sec=0.05)

            image = Image()
            image.header.frame_id = "camera_link"
            image.header.stamp.sec = 0
            image.header.stamp.nanosec = 0
            image.height = 480
            image.width = 640
            image.encoding = "rgb8"
            image.is_bigendian = 0
            image.step = 640*3
            image.data = []

            info = CameraInfo()
            info.header.frame_id = "camera_link"
            info.header.stamp.sec = 0
            info.header.stamp.nanosec = 0
            info.height = 480
            info.width = 640

            image_publisher.publish(image)
            info_publisher.publish(info)
            output_deadline = time.monotonic()+3.0
            while not result_messages or not status_messages:
                if time.monotonic() >= output_deadline:
                    self.fail("adapter did not publish result and status")
                executor.spin_once(timeout_sec=0.05)

            return json.loads(result_messages[-1]), json.loads(status_messages[-1])
        finally:
            executor.remove_node(adapter)
            executor.remove_node(probe)
            adapter.destroy_node()
            probe.destroy_node()

    def test_mock_publishes_valid_empty_contract_bound_to_camera_capture(self):
        result, status = self._exercise_backend("mock")
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["result"]["detections"], [])
        self.assertEqual(result["result"]["capture"], {
            "stamp_ns": 0,
            "clock_domain": "ros_sim",
            "frame_id": "camera_link",
            "width": 640,
            "height": 480,
        })
        self.assertEqual(status["state"], "empty")
        self.assertEqual(status["detection_count"], 0)
        self.assertNotIn("cmd_vel", result)

    def test_off_publishes_unavailable_without_detections(self):
        result, status = self._exercise_backend("off")
        self.assertEqual(result["status"], "unavailable")
        self.assertEqual(result["error"]["code"], "vision_disabled")
        self.assertEqual(status["state"], "unavailable")
        self.assertEqual(status["detection_count"], 0)

    def test_service_backend_normalizes_fake_http_detection_asynchronously(self):
        response_payload = json.dumps({
            "success": True,
            "moduleId": "ObjectDetectionYOLOv11",
            "inferenceMs": 2,
            "processMs": 3,
            "analysisRoundTripMs": 4,
            "predictions": [{"label": "person", "confidence": 0.9,
                             "x_min": 0, "y_min": 0, "x_max": 2, "y_max": 1}],
        }).encode("utf-8")
        requests = []

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                length = int(self.headers.get("Content-Length", "0"))
                requests.append((self.headers.get("Content-Type"), self.rfile.read(length)))
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(response_payload)))
                self.end_headers()
                self.wfile.write(response_payload)

            def log_message(self, *_args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server_thread = Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        suffix = "/vision_adapter_test/service"
        result_messages = []
        status_messages = []
        parameters = [
            Parameter("backend", value="service"),
            Parameter("use_sim_time", value=True),
            Parameter("clock_domain", value="ros_sim"),
            Parameter("image_topic", value=suffix+"/image"),
            Parameter("camera_info_topic", value=suffix+"/camera_info"),
            Parameter("result_topic", value=suffix+"/result"),
            Parameter("status_topic", value=suffix+"/status"),
            Parameter("service_endpoint_url",
                      value=f"http://127.0.0.1:{server.server_port}/detect"),
            Parameter("service_model_version", value="test-weights-v1"),
            Parameter("service_labels_version", value="test-labels-v1"),
            Parameter("service_label_map_json", value='{"0":"person"}'),
            Parameter("service_timeout_sec", value=1.0),
            Parameter("service_max_age_ns", value=1_000_000_000),
            Parameter("service_min_interval_sec", value=0.6),
            Parameter("service_max_requests", value=1),
        ]
        adapter = probe = None
        executor = SingleThreadedExecutor()
        try:
            adapter = VisionAdapterNode(parameter_overrides=parameters)
            probe = Node("vision_service_probe")
            image_publisher = probe.create_publisher(
                Image, suffix+"/image", qos_profile_sensor_data)
            info_publisher = probe.create_publisher(
                CameraInfo, suffix+"/camera_info", qos_profile_sensor_data)
            clock_publisher = probe.create_publisher(Clock, "/clock", 10)
            probe.create_subscription(String, suffix+"/result",
                                      lambda msg: result_messages.append(msg.data), 10)
            probe.create_subscription(String, suffix+"/status",
                                      lambda msg: status_messages.append(msg.data), 10)
            executor.add_node(adapter)
            executor.add_node(probe)
            discovery_deadline = time.monotonic()+3.0
            while (image_publisher.get_subscription_count() == 0
                   or info_publisher.get_subscription_count() == 0):
                if time.monotonic() >= discovery_deadline:
                    self.fail("service adapter camera subscriptions were not discovered")
                executor.spin_once(timeout_sec=0.05)

            image = Image()
            image.header.frame_id = "camera_link"
            image.header.stamp.sec = 1
            image.header.stamp.nanosec = 0
            image.height = 1
            image.width = 2
            image.encoding = "rgb8"
            image.step = 6
            image.data = [10, 20, 30, 40, 50, 60]
            info = CameraInfo()
            info.header.frame_id = "camera_link"
            info.header.stamp.sec = 1
            info.header.stamp.nanosec = 0
            info.height = 1
            info.width = 2
            image_publisher.publish(image)
            info_publisher.publish(info)

            output_deadline = time.monotonic()+3.0
            clock_started = time.monotonic()
            while not result_messages or not status_messages:
                if time.monotonic() >= output_deadline:
                    self.fail("service adapter did not publish result and status")
                sim_ns = 1_000_000_000+int((time.monotonic()-clock_started)*1_000_000_000)
                clock_message = Clock()
                clock_message.clock.sec = sim_ns//1_000_000_000
                clock_message.clock.nanosec = sim_ns%1_000_000_000
                clock_publisher.publish(clock_message)
                executor.spin_once(timeout_sec=0.05)

            result = json.loads(result_messages[-1])
            status = json.loads(status_messages[-1])
            self.assertEqual(result["status"], "ok")
            self.assertEqual(result["result"]["capture"]["clock_domain"], "ros_sim")
            self.assertEqual(result["result"]["detections"][0]["class_id"], 0)
            self.assertEqual(result["result"]["detections"][0]["label"], "person")
            self.assertEqual(status["state"], "ok")
            self.assertEqual(len(requests), 1)
            self.assertIn(b'name="image"', requests[0][1])
            self.assertNotIn("cmd_vel", result)

            image.header.stamp.sec = 3
            info.header.stamp.sec = 3
            image_publisher.publish(image)
            info_publisher.publish(info)
            second_pair_deadline = time.monotonic()+0.75
            while time.monotonic() < second_pair_deadline:
                sim_ns = 3_000_000_000+int(
                    (time.monotonic()-second_pair_deadline+0.75)*1_000_000_000)
                clock_message = Clock()
                clock_message.clock.sec = sim_ns//1_000_000_000
                clock_message.clock.nanosec = sim_ns%1_000_000_000
                clock_publisher.publish(clock_message)
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(len(requests), 1)
        finally:
            if adapter is not None:
                if adapter in executor.get_nodes():
                    executor.remove_node(adapter)
                adapter.destroy_node()
            if probe is not None:
                if probe in executor.get_nodes():
                    executor.remove_node(probe)
                probe.destroy_node()
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=2)

    def test_zmq_backend_normalizes_fake_71_detection_asynchronously(self):
        try:
            import zmq
        except ImportError:
            self.skipTest("pyzmq is required")
        context = zmq.Context()
        pull = context.socket(zmq.PULL)
        rep = context.socket(zmq.REP)
        image_port = pull.bind_to_random_port("tcp://127.0.0.1")
        reply_port = rep.bind_to_random_port("tcp://127.0.0.1")
        received = []

        def serve_once():
            pull.setsockopt(zmq.RCVTIMEO, 5000)
            rep.setsockopt(zmq.RCVTIMEO, 5000)
            try:
                received.append(pull.recv())
                received.append(rep.recv_string())
                rep.send_json({"detections": [
                    {"box": [0.0, 0.0, 2.0, 1.0], "label": "person",
                     "confidence": 0.9}]})
            except zmq.error.ZMQError:
                pass

        server_thread = Thread(target=serve_once, daemon=True)
        server_thread.start()
        suffix = "/vision_adapter_test/zmq"
        result_messages = []
        parameters = [
            Parameter("backend", value="zmq"),
            Parameter("use_sim_time", value=True),
            Parameter("image_topic", value=suffix+"/image"),
            Parameter("camera_info_topic", value=suffix+"/camera_info"),
            Parameter("result_topic", value=suffix+"/result"),
            Parameter("status_topic", value=suffix+"/status"),
            Parameter("zmq_image_endpoint", value=f"tcp://127.0.0.1:{image_port}"),
            Parameter("zmq_reply_endpoint", value=f"tcp://127.0.0.1:{reply_port}"),
            Parameter("zmq_model_version", value="test-weights-v1"),
            Parameter("zmq_labels_version", value="test-labels-v1"),
            Parameter("zmq_label_map_json", value='{"0":"person"}'),
            Parameter("zmq_timeout_sec", value=2.0),
            Parameter("service_max_age_ns", value=1_000_000_000),
            Parameter("service_max_requests", value=1),
        ]
        adapter = probe = None
        executor = SingleThreadedExecutor()
        try:
            adapter = VisionAdapterNode(parameter_overrides=parameters)
            probe = Node("vision_zmq_probe")
            image_publisher = probe.create_publisher(
                Image, suffix+"/image", qos_profile_sensor_data)
            info_publisher = probe.create_publisher(
                CameraInfo, suffix+"/camera_info", qos_profile_sensor_data)
            clock_publisher = probe.create_publisher(Clock, "/clock", 10)
            probe.create_subscription(String, suffix+"/result",
                                      lambda msg: result_messages.append(msg.data), 10)
            executor.add_node(adapter)
            executor.add_node(probe)
            deadline = time.monotonic()+3.0
            while (image_publisher.get_subscription_count() == 0
                   or info_publisher.get_subscription_count() == 0):
                if time.monotonic() >= deadline:
                    self.fail("zmq adapter camera subscriptions were not discovered")
                executor.spin_once(timeout_sec=0.05)
            image = Image()
            image.header.frame_id = "camera_link"
            image.header.stamp.sec = 1
            image.height, image.width, image.encoding, image.step = 1, 2, "rgb8", 6
            image.data = [10, 20, 30, 40, 50, 60]
            info = CameraInfo()
            info.header.frame_id = "camera_link"
            info.header.stamp.sec = 1
            info.height, info.width = 1, 2
            image_publisher.publish(image)
            info_publisher.publish(info)
            started = time.monotonic()
            while not result_messages:
                if time.monotonic()-started >= 4.0:
                    self.fail("zmq adapter did not publish a result")
                sim_ns = 1_000_000_000+int((time.monotonic()-started)*1_000_000_000)
                clock_message = Clock()
                clock_message.clock.sec = sim_ns//1_000_000_000
                clock_message.clock.nanosec = sim_ns%1_000_000_000
                clock_publisher.publish(clock_message)
                executor.spin_once(timeout_sec=0.05)
            result = json.loads(result_messages[-1])
            self.assertEqual(result["status"], "ok")
            self.assertEqual(result["result"]["detections"][0]["label"], "person")
            self.assertEqual(received[1], "detect")
            self.assertTrue(received[0].startswith(b"P6\n2 1\n255\n"))
        finally:
            if adapter is not None:
                if adapter in executor.get_nodes():
                    executor.remove_node(adapter)
                adapter.destroy_node()
            if probe is not None:
                if probe in executor.get_nodes():
                    executor.remove_node(probe)
                probe.destroy_node()
            server_thread.join(timeout=6)
            pull.close(linger=0)
            rep.close(linger=0)
            context.term()


if __name__ == "__main__":
    unittest.main()
