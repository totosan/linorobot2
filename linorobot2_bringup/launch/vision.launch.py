from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='off',
            choices=['off', 'mock'],
            description='Use only the G5 off/mock backends; live service inference is excluded',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='USB camera image topic',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='USB camera calibration topic',
        ),
        Node(
            package='linorobot2_vision',
            executable='vision_adapter_node',
            name='vision_adapter',
            output='screen',
            parameters=[{
                'backend': LaunchConfiguration('backend'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'clock_domain': 'unix_utc',
                'expected_frame_id': 'usb_cam',
            }],
        ),
    ])