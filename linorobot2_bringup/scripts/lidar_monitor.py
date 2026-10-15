#!/usr/bin/env python3
"""Fail-closed ROS monitor for the required LaserScan stream."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from lidar_monitor_contract import ScanHealth


class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        self.failure_reason = None
        self.health = ScanHealth(time.monotonic())
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )
        self.timer = self.create_timer(2.0, self.check_scan_status)
        self.get_logger().info(
            'Monitoring /scan; startup grace 10s, stale timeout 15s'
        )

    def scan_callback(self, _message):
        was_first_scan = self.health.last_scan_time is None
        self.health.observe_scan(time.monotonic())
        if was_first_scan:
            self.get_logger().info('First scan received; lidar monitoring active')

    def check_scan_status(self):
        reason = self.health.failure_reason(time.monotonic())
        if reason is None or self.failure_reason is not None:
            return

        self.failure_reason = reason
        self.get_logger().fatal(f'Required lidar failed: {reason}')
        # The main thread observes this after the callback returns, then shuts
        # down only this process's ROS context. Launch handles the process exit.


def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and node.failure_reason is None:
            executor.spin_once(timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        failed = node.failure_reason is not None
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
