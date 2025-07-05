#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import sys
import time
import os
import signal
import subprocess
import threading

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.last_scan_time = None
        self.timeout = 15.0  # 15 seconds timeout to allow startup
        self.startup_grace_period = 10.0  # 10 seconds for initial startup
        self.start_time = time.time()
        self.scan_received = False
        self.shutdown_initiated = False
        
        # Timer to check if we're receiving scan data
        self.timer = self.create_timer(2.0, self.check_scan_status)
        self.get_logger().info('Lidar monitor started - monitoring /scan topic')
        self.get_logger().info(f'Grace period: {self.startup_grace_period}s, Timeout: {self.timeout}s')

    def scan_callback(self, msg):
        self.last_scan_time = time.time()
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info('First scan data received - monitoring active')

    def emergency_shutdown_worker(self, reason):
        """Worker function to handle shutdown in a separate thread"""
        print(f"[EMERGENCY] {reason}")
        print("[EMERGENCY] Initiating system shutdown...")
        
        # Get process info
        current_pid = os.getpid()
        parent_pid = os.getppid()
        
        print(f"[EMERGENCY] Current PID: {current_pid}, Parent PID: {parent_pid}")
        
        # Use killall to terminate ros2 launch processes
        try:
            print("[EMERGENCY] Using killall to terminate ros2 processes...")
            subprocess.run(['killall', '-9', 'ros2'], check=False)
            subprocess.run(['killall', '-9', 'python3'], check=False)
        except Exception as e:
            print(f"[EMERGENCY] killall failed: {e}")
        
        # Try direct process termination
        try:
            print(f"[EMERGENCY] Killing parent process {parent_pid}")
            os.kill(parent_pid, signal.SIGKILL)
        except Exception as e:
            print(f"[EMERGENCY] Failed to kill parent: {e}")
            
        # Force kill process group
        try:
            print("[EMERGENCY] Killing process group")
            os.killpg(0, signal.SIGKILL)
        except Exception as e:
            print(f"[EMERGENCY] Failed to kill process group: {e}")
            
        # Final fallback
        print("[EMERGENCY] Final exit")
        os._exit(1)

    def emergency_shutdown(self, reason):
        """Force shutdown of the entire launch system"""
        if self.shutdown_initiated:
            return
        self.shutdown_initiated = True
        
        self.get_logger().error(f'EMERGENCY SHUTDOWN: {reason}')
        
        # Start shutdown in a separate thread to avoid getting killed mid-process
        shutdown_thread = threading.Thread(target=self.emergency_shutdown_worker, args=(reason,))
        shutdown_thread.daemon = True
        shutdown_thread.start()
        
        # Give the thread time to work
        time.sleep(1)
        
        # Force exit this process
        os._exit(1)

    def check_scan_status(self):
        current_time = time.time()
        time_since_start = current_time - self.start_time
        
        # During startup grace period, just wait
        if time_since_start < self.startup_grace_period:
            if not self.scan_received:
                self.get_logger().info(f'Waiting for lidar startup... ({time_since_start:.1f}s/{self.startup_grace_period}s)')
                return
        
        # After grace period, check if we ever received data
        if not self.scan_received and time_since_start >= self.startup_grace_period:
            self.emergency_shutdown(f'No scan data received after {self.startup_grace_period}s startup period - Lidar failed to start')
            
        # If we have received data, check for timeout
        if self.scan_received and self.last_scan_time:
            time_since_last_scan = current_time - self.last_scan_time
            
            if time_since_last_scan > self.timeout:
                self.emergency_shutdown(f'No scan data received for {time_since_last_scan:.1f} seconds - Lidar appears to have stopped')
            else:
                self.get_logger().debug(f'Scan data received {time_since_last_scan:.1f}s ago')

def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error in lidar monitor: {e}')
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
