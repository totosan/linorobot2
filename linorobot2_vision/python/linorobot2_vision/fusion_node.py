"""ROS 2 adapter for capture-time 2D camera/LiDAR association."""

from collections import OrderedDict
import json

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .contract import VisionContractError
from .lidar_camera_fusion import fuse_scan_with_detections


_CACHE_SIZE = 20


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _transform_tuple(transform_stamped):
    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation
    return (
        translation.x, translation.y, translation.z,
        rotation.x, rotation.y, rotation.z, rotation.w,
    )


class LidarCameraFusionNode(Node):
    """Join detector JSON, CameraInfo, LaserScan, and TF by capture time."""

    def __init__(self):
        super().__init__("lidar_camera_fusion")
        self.declare_parameter("result_topic", "/vision/result")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("fused_topic", "/vision/fused_planar")
        self.declare_parameter("markers_topic", "/vision/fused_planar_markers")
        self.declare_parameter("projection_frame", "camera_optical_frame")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("scan_slop_ns", 50_000_000)
        self.declare_parameter("camera_info_slop_ns", 50_000_000)
        self.declare_parameter("max_result_age_ns", 1_000_000_000)
        self.declare_parameter("cluster_range_jump_m", 0.15)

        self.scan_slop_ns = self.get_parameter("scan_slop_ns").value
        self.camera_info_slop_ns = self.get_parameter("camera_info_slop_ns").value
        self.max_result_age_ns = self.get_parameter("max_result_age_ns").value
        self.cluster_range_jump_m = self.get_parameter("cluster_range_jump_m").value
        if self.get_parameter("output_frame").value != "base_link":
            raise ValueError("the current planar output is defined only in base_link")
        for name, value in (("scan_slop_ns", self.scan_slop_ns),
                            ("camera_info_slop_ns", self.camera_info_slop_ns),
                            ("max_result_age_ns", self.max_result_age_ns)):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if (isinstance(self.cluster_range_jump_m, bool)
                or not isinstance(self.cluster_range_jump_m, (int, float))
                or self.cluster_range_jump_m <= 0):
            raise ValueError("cluster_range_jump_m must be positive")

        self.scans = OrderedDict()
        self.camera_infos = OrderedDict()
        self.pending_results = OrderedDict()
        self.publisher = self.create_publisher(
            String, self.get_parameter("fused_topic").value, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.get_parameter("markers_topic").value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            String, self.get_parameter("result_topic").value,
            self._on_result, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self._process_pending)
        self.get_logger().info(
            "capture-time LiDAR/image association ready; output is planar only")

    @staticmethod
    def _remember(cache, stamp, message):
        cache[stamp] = message
        cache.move_to_end(stamp)
        while len(cache) > _CACHE_SIZE:
            cache.popitem(last=False)

    def _on_scan(self, message):
        self._remember(self.scans, _stamp_ns(message.header.stamp), message)

    def _on_camera_info(self, message):
        self._remember(self.camera_infos, _stamp_ns(message.header.stamp), message)

    def _on_result(self, message):
        try:
            response = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self._publish_status("invalid_result_json", str(exc))
            return
        if not isinstance(response, dict) or response.get("status") != "ok":
            state = response.get("status", "invalid_result") if isinstance(response, dict) else "invalid_result"
            self._publish_status(state, "no successful detection payload to fuse")
            return
        try:
            capture = response["result"]["capture"]
            stamp = capture["stamp_ns"]
            if isinstance(stamp, bool) or not isinstance(stamp, int):
                raise VisionContractError("capture.stamp_ns must be an integer")
        except (KeyError, TypeError, VisionContractError) as exc:
            self._publish_status("invalid_capture", str(exc))
            return
        self._remember(self.pending_results, stamp, response)
        self._process_pending()

    @staticmethod
    def _nearest(cache, stamp, slop_ns):
        if not cache:
            return None
        candidate_stamp = min(cache, key=lambda value: abs(value - stamp))
        if abs(candidate_stamp - stamp) > slop_ns:
            return None
        return candidate_stamp, cache[candidate_stamp]

    def _process_pending(self):
        now_ns = self.get_clock().now().nanoseconds
        for capture_stamp, response in list(self.pending_results.items()):
            capture = response["result"]["capture"]
            if capture_stamp > now_ns + self.scan_slop_ns:
                self._publish_status("future_capture",
                                     "capture stamp is ahead of simulated ROS time",
                                     capture_stamp)
                del self.pending_results[capture_stamp]
                continue
            info_entry = self._nearest(
                self.camera_infos, capture_stamp, self.camera_info_slop_ns)
            scan_entry = self._nearest(self.scans, capture_stamp, self.scan_slop_ns)
            if info_entry is not None and scan_entry is not None:
                info_stamp, info = info_entry
                scan_stamp, scan = scan_entry
                try:
                    projection_frame = self.get_parameter("projection_frame").value
                    output_frame = self.get_parameter("output_frame").value
                    time = Time(nanoseconds=scan_stamp, clock_type=self.get_clock().clock_type)
                    camera_transform = self.tf_buffer.lookup_transform(
                        projection_frame, scan.header.frame_id, time,
                        timeout=Duration(seconds=0.05))
                    base_transform = self.tf_buffer.lookup_transform(
                        output_frame, scan.header.frame_id, time,
                        timeout=Duration(seconds=0.05))
                    camera_mapping = {
                        "stamp_ns": info_stamp,
                        "frame_id": info.header.frame_id,
                        "width": info.width,
                        "height": info.height,
                        "projection": list(info.p),
                        "distortion": list(info.d),
                    }
                    scan_mapping = {
                        "stamp_ns": scan_stamp,
                        "frame_id": scan.header.frame_id,
                        "angle_min": scan.angle_min,
                        "angle_increment": scan.angle_increment,
                        "range_min": scan.range_min,
                        "range_max": scan.range_max,
                        "ranges": list(scan.ranges),
                    }
                    fused = fuse_scan_with_detections(
                        response, camera_mapping, scan_mapping,
                        _transform_tuple(camera_transform),
                        _transform_tuple(base_transform),
                        max_scan_skew_ns=self.scan_slop_ns,
                        max_camera_info_skew_ns=self.camera_info_slop_ns,
                        cluster_range_jump_m=self.cluster_range_jump_m)
                    fused["output_frame"] = output_frame
                    self._publish(fused)
                    del self.pending_results[capture_stamp]
                    continue
                except TransformException as exc:
                    # Keep the result briefly in case TF arrives after sensor data.
                    if now_ns - capture_stamp <= self.max_result_age_ns:
                        continue
                    self._publish_status("tf_unavailable", str(exc), capture_stamp)
                    del self.pending_results[capture_stamp]
                    continue
                except (VisionContractError, ValueError, TypeError) as exc:
                    self._publish_status("fusion_rejected", str(exc), capture_stamp)
                    del self.pending_results[capture_stamp]
                    continue

            if now_ns - capture_stamp > self.max_result_age_ns:
                self._publish_status("inputs_unavailable_or_stale",
                                     "no synchronized scan and CameraInfo before expiry",
                                     capture_stamp)
                del self.pending_results[capture_stamp]

    def _publish_status(self, state, message, capture_stamp_ns=None):
        payload = {"schema_version": "1.0", "state": state, "message": message}
        if capture_stamp_ns is not None:
            payload["capture_stamp_ns"] = capture_stamp_ns
        self._publish(payload)

    def _publish(self, payload):
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.publisher.publish(message)
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        capture = payload.get("capture", {})
        stamp_ns = payload.get("scan", {}).get(
            "stamp_ns", capture.get("stamp_ns", 0))
        output_frame = payload.get("output_frame", self.get_parameter("output_frame").value)
        for index, detection in enumerate(payload.get("detections", [])):
            lidar = detection.get("lidar", {})
            point = lidar.get("point_base_link_m")
            if point is None:
                continue
            marker = Marker()
            marker.header.frame_id = output_frame
            marker.header.stamp.sec = stamp_ns // 1_000_000_000
            marker.header.stamp.nanosec = stamp_ns % 1_000_000_000
            marker.ns = "vision_lidar_fusion"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = float(point[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.08
            if lidar.get("status") == "matched":
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.9, 0.2
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.55, 0.05
            marker.color.a = 1.0
            marker.lifetime.sec = 1
            markers.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = marker.ns + "_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(point[0])
            label.pose.position.y = float(point[1])
            label.pose.position.z = float(point[2]) + 0.12
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = "%s %s %.2fm" % (
                detection.get("label", "object"), lidar.get("status", "unknown"),
                lidar.get("range_m", 0.0))
            label.lifetime.sec = 1
            markers.markers.append(label)
        self.marker_publisher.publish(markers)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = LidarCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
