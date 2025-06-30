#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from linorobot2_vision.msg import DetectedObjectArray, DetectedObject
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import colorsys
import math

class ObjectVisualizationNode(Node):
    def __init__(self):
        super().__init__('object_visualization')
        
        # QoS profile for reliable communication with detected objects
        objects_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS profile for visualization markers (compatible with RViz)
        markers_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber to detected objects
        self.objects_sub = self.create_subscription(
            DetectedObjectArray,
            'vision/objects',
            self.objects_callback,
            objects_qos_profile
        )
        
        # Alternative subscription with different QoS for testing
        # (comment out the above subscription and uncomment this if needed)
        # from rclpy.qos import qos_profile_sensor_data
        # self.objects_sub = self.create_subscription(
        #     DetectedObjectArray,
        #     'vision/objects',
        #     self.objects_callback,
        #     qos_profile_sensor_data
        # )
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'vision/object_markers',
            markers_qos_profile
        )
        
        # Parameters
        self.marker_lifetime = self.declare_parameter('marker_lifetime', 1.0).value
        self.text_size = self.declare_parameter('text_size', 0.2).value
        self.point_size = self.declare_parameter('point_size', 0.05).value
        self.show_points = self.declare_parameter('show_points', True).value
        self.show_labels = self.declare_parameter('show_labels', True).value
        self.show_distance = self.declare_parameter('show_distance', True).value
        # Verbosity parameter
        self.verbose = self.declare_parameter('verbose', True).value
        
        # Color mapping for different object classes
        self.class_colors = {}
        self.color_index = 0
        
        # Add debugging timer to check subscription status
        self.debug_timer = self.create_timer(5.0, self.debug_status)
        self.message_count = 0
        self.last_message_time = None
        
        self.get_logger().info('Object Visualization Node started')
        if self.verbose:
            self.get_logger().info(f'Subscribed to topic: vision/objects')
            self.get_logger().info(f'Publishing markers to: vision/object_markers')
            self.get_logger().info(f'Marker lifetime: {self.marker_lifetime}s')
            self.get_logger().info(f'Show points: {self.show_points}')
            self.get_logger().info(f'Show labels: {self.show_labels}')
            self.get_logger().info(f'Show distance: {self.show_distance}')
            self.get_logger().info('Objects QoS: RELIABLE, VOLATILE, KEEP_LAST(10)')
            self.get_logger().info('Markers QoS: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(10)')

    def debug_status(self):
        """Debug timer callback to check subscription status"""
        current_time = self.get_clock().now()
        
        # Log subscription status
        if self.verbose:
            self.get_logger().info(f'Subscription status - Messages received: {self.message_count}')
        if self.last_message_time:
            time_since_last = (current_time - self.last_message_time).nanoseconds / 1e9
            if self.verbose:
                self.get_logger().info(f'Time since last message: {time_since_last:.1f}s')
        else:
            if self.verbose:
                self.get_logger().warn('No messages received yet on vision/objects topic')
                self.get_logger().info('Check if the publisher node is running and publishing to /vision/objects')

    def get_color_for_class(self, class_name):
        """Get a consistent color for each object class"""
        if class_name not in self.class_colors:
            # Generate a color using HSV color space for better distribution
            hue = (self.color_index * 137.5) % 360  # Golden angle approximation
            self.color_index += 1
            
            # Convert HSV to RGB
            r, g, b = colorsys.hsv_to_rgb(hue/360.0, 0.8, 0.9)
            
            color = ColorRGBA()
            color.r = float(r)
            color.g = float(g)
            color.b = float(b)
            color.a = 0.8
            
            self.class_colors[class_name] = color
            if self.verbose:
                self.get_logger().info(f'Assigned color to class "{class_name}": RGB({r:.2f}, {g:.2f}, {b:.2f})')
        
        return self.class_colors[class_name]

    def create_points_marker(self, obj, marker_id, frame_id):
        """Create a marker for the LiDAR points of an object"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"object_points_{obj.name}"
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = self.point_size
        marker.scale.y = self.point_size
        marker.scale.z = self.point_size
        
        # Set color
        color = self.get_color_for_class(obj.name)
        marker.color = color
        
        # Add points
        marker.points = obj.points
        
        # Set lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        return marker

    def create_text_marker(self, obj, marker_id, frame_id):
        """Create a text marker for object label and distance"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"object_labels_{obj.name}"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Calculate center position from points
        if len(obj.points) > 0:
            center_x = sum(p.x for p in obj.points) / len(obj.points)
            center_y = sum(p.y for p in obj.points) / len(obj.points)
            center_z = sum(p.z for p in obj.points) / len(obj.points) + 0.3  # Offset above points
        else:
            center_x, center_y, center_z = 0.0, 0.0, 0.3
        
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = center_z
        marker.pose.orientation.w = 1.0
        
        # Set text content
        text_parts = []
        if self.show_labels:
            text_parts.append(f"{obj.name}")
        if self.show_distance:
            text_parts.append(f"{obj.distance:.2f}m")
        
        marker.text = "\n".join(text_parts)
        
        # Set text properties
        marker.scale.z = self.text_size
        
        # Set color (white with some transparency)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        
        # Set lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        return marker

    def create_bounding_box_marker(self, obj, marker_id, frame_id):
        """Create a bounding box marker around the object points"""
        if len(obj.points) == 0:
            return None
            
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"object_bbox_{obj.name}"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Calculate bounding box
        min_x = min(p.x for p in obj.points)
        max_x = max(p.x for p in obj.points)
        min_y = min(p.y for p in obj.points)
        max_y = max(p.y for p in obj.points)
        min_z = min(p.z for p in obj.points)
        max_z = max(p.z for p in obj.points)
        
        # Create bounding box corners (bottom face, then top face, then connecting lines)
        bbox_points = []
        
        # Bottom face
        bbox_points.extend([
            Point(x=min_x, y=min_y, z=min_z),
            Point(x=max_x, y=min_y, z=min_z),
            Point(x=max_x, y=max_y, z=min_z),
            Point(x=min_x, y=max_y, z=min_z),
            Point(x=min_x, y=min_y, z=min_z),  # Close the bottom face
        ])
        
        # Top face
        bbox_points.extend([
            Point(x=min_x, y=min_y, z=max_z),
            Point(x=max_x, y=min_y, z=max_z),
            Point(x=max_x, y=max_y, z=max_z),
            Point(x=min_x, y=max_y, z=max_z),
            Point(x=min_x, y=min_y, z=max_z),  # Close the top face
        ])
        
        marker.points = bbox_points
        
        # Set line properties
        marker.scale.x = 0.02  # Line width
        
        # Set color
        color = self.get_color_for_class(obj.name)
        marker.color = color
        
        # Set lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        return marker

    def objects_callback(self, msg):
        """Callback for detected objects"""
        self.message_count += 1
        self.last_message_time = self.get_clock().now()
        if self.verbose:
            self.get_logger().info(f'[CALLBACK] Received message #{self.message_count} with {len(msg.objects)} objects')
            self.get_logger().info(f'[CALLBACK] Message frame_id: {msg.header.frame_id}')
            self.get_logger().info(f'[CALLBACK] Message timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        if len(msg.objects) == 0:
            if self.verbose:
                self.get_logger().warn('[CALLBACK] Message contains no objects')
            return
        
        marker_array = MarkerArray()
        marker_id = 0
        
        for i, obj in enumerate(msg.objects):
            if self.verbose:
                self.get_logger().info(f'[CALLBACK] Object {i}: name="{obj.name}", distance={obj.distance:.2f}m, points={len(obj.points)}')
            
            # Create points marker if enabled and points exist
            if self.show_points and len(obj.points) > 0:
                points_marker = self.create_points_marker(obj, marker_id, msg.header.frame_id)
                marker_array.markers.append(points_marker)
                marker_id += 1
                if self.verbose:
                    self.get_logger().info(f'[CALLBACK] Created points marker for {obj.name}')
            
            # Create bounding box marker if points exist
            if len(obj.points) > 0:
                bbox_marker = self.create_bounding_box_marker(obj, marker_id, msg.header.frame_id)
                if bbox_marker:
                    marker_array.markers.append(bbox_marker)
                    marker_id += 1
                    if self.verbose:
                        self.get_logger().info(f'[CALLBACK] Created bbox marker for {obj.name}')
            
            # Create text marker if enabled
            if self.show_labels or self.show_distance:
                text_marker = self.create_text_marker(obj, marker_id, msg.header.frame_id)
                marker_array.markers.append(text_marker)
                marker_id += 1
                if self.verbose:
                    self.get_logger().info(f'[CALLBACK] Created text marker for {obj.name}')
        
        # Publish markers
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            if self.verbose:
                self.get_logger().info(f'[CALLBACK] Published {len(marker_array.markers)} markers to vision/object_markers')
        else:
            if self.verbose:
                self.get_logger().warn('[CALLBACK] No markers to publish')

def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectVisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
