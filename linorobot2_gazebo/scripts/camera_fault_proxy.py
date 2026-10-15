#!/usr/bin/env python3
"""Simulation-only gate for injecting a camera-image topic outage in G3."""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class CameraFaultProxy(Node):
    def __init__(self):
        super().__init__("g3_camera_fault_proxy")
        self.enabled = True
        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.subscription = self.create_subscription(
            Image, "/g3_fault_injector/camera/image_raw_source", self.on_image, 10)
        self.service = self.create_service(
            SetBool, "/g3_fault_injector/set_camera_enabled", self.set_enabled)

    def on_image(self, message):
        if self.enabled:
            self.publisher.publish(message)

    def set_enabled(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = "camera image forwarding enabled" if self.enabled else "camera image forwarding disabled"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraFaultProxy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
