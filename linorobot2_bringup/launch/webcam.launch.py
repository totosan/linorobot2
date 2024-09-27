import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='sensor', 
            default_value='logitech',
            description='WebCam Logitech C920'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='webcam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg2rgb',
                'camera_frame_id': 'usb_cam',
                # Add other parameters as needed
            }]#,
            #remappings=[
            #    ('/camera/camera_info', '/camera/color/camera_info'),
            #    ('/camera/image_raw', '/camera/color/image_raw'),
            #    ('/camera/points', '/camera/depth/color/points'),
            #    ('/camera/depth/image_raw', '/camera/depth/image_rect_raw'),
            #]
        ),
    ])