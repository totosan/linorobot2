"""Publish camera frames with capture-matched contract-v1 2D boxes."""

from collections import OrderedDict
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .contract import VisionContractError
from .image_overlay import boxes_for_capture, draw_bbox_overlay


_CACHE_SIZE = 8


def _stamp_ns(image):
    stamp = image.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class VisionImageOverlayNode(Node):
    """Keep the live image flowing and overlay only exact-frame detections."""

    def __init__(self):
        super().__init__("vision_image_overlay")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("result_topic", "/vision/result")
        self.declare_parameter("overlay_topic", "/vision/overlay/image_raw")
        self.declare_parameter("line_thickness", 3)
        self.images = OrderedDict()
        self.results = OrderedDict()
        self.publisher = self.create_publisher(
            Image, self.get_parameter("overlay_topic").value, 5)
        self.image_subscription = self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self._on_image, qos_profile_sensor_data)
        self.result_subscription = self.create_subscription(
            String, self.get_parameter("result_topic").value,
            self._on_result, 10)
        self.get_logger().info(
            "publishing capture-matched 2D overlays; image passthrough remains live")

    @staticmethod
    def _trim(cache):
        while len(cache) > _CACHE_SIZE:
            cache.popitem(last=False)

    def _on_image(self, image):
        stamp = _stamp_ns(image)
        self.images[stamp] = image
        self.images.move_to_end(stamp)
        self._trim(self.images)
        # Keep the preview live even when no vision result is available.
        self.publisher.publish(image)
        response = self.results.pop(stamp, None)
        if response is not None:
            self._publish_overlay(image, response)

    def _on_result(self, message):
        try:
            response = json.loads(message.data)
            # Full schema validation occurs in boxes_for_capture.
            if not isinstance(response, dict) or response.get("status") != "ok":
                return
            capture = response.get("result", {}).get("capture", {})
            stamp = capture.get("stamp_ns")
            if isinstance(stamp, bool) or not isinstance(stamp, int):
                return
            image = self.images.get(stamp)
            if image is None:
                self.results[stamp] = response
                self.results.move_to_end(stamp)
                self._trim(self.results)
                return
            self._publish_overlay(image, response)
        except (json.JSONDecodeError, AttributeError, TypeError, VisionContractError) as exc:
            self.get_logger().warning(f"ignored invalid vision result: {exc}")

    def _publish_overlay(self, image, response):
        try:
            boxes = boxes_for_capture(
                response,
                stamp_ns=_stamp_ns(image),
                frame_id=image.header.frame_id,
                width=image.width,
                height=image.height)
            if boxes is None:
                return
            data = draw_bbox_overlay(
                image.data,
                width=image.width,
                height=image.height,
                step=image.step,
                encoding=image.encoding,
                boxes=boxes,
                thickness=self.get_parameter("line_thickness").value)
        except (TypeError, ValueError, VisionContractError) as exc:
            self.get_logger().warning(f"could not draw vision overlay: {exc}")
            return

        overlay = Image()
        overlay.header = image.header
        overlay.height = image.height
        overlay.width = image.width
        overlay.encoding = image.encoding
        overlay.is_bigendian = image.is_bigendian
        overlay.step = image.step
        overlay.data = data
        self.publisher.publish(overlay)


def main(args=None):
    rclpy.init(args=args)
    node = VisionImageOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
