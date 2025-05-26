#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'marker_lifetime',
            default_value='2.0',
            description='Lifetime of markers in seconds'
        ),
        DeclareLaunchArgument(
            'text_size',
            default_value='0.2',
            description='Size of text labels'
        ),
        DeclareLaunchArgument(
            'point_size',
            default_value='0.05',
            description='Size of LiDAR points'
        ),
        DeclareLaunchArgument(
            'show_points',
            default_value='true',
            description='Whether to show LiDAR points'
        ),
        DeclareLaunchArgument(
            'show_labels',
            default_value='true',
            description='Whether to show object labels'
        ),
        DeclareLaunchArgument(
            'show_distance',
            default_value='true',
            description='Whether to show distance information'
        ),
        
        # Object visualization node
        Node(
            package='linorobot2_vision',
            executable='object_visualization_node',
            name='object_visualization',
            output='screen',
            parameters=[{
                'marker_lifetime': LaunchConfiguration('marker_lifetime'),
                'text_size': LaunchConfiguration('text_size'),
                'point_size': LaunchConfiguration('point_size'),
                'show_points': LaunchConfiguration('show_points'),
                'show_labels': LaunchConfiguration('show_labels'),
                'show_distance': LaunchConfiguration('show_distance'),
            }],
            remappings=[
                # You can remap topics here if needed
                # ('vision/objects', '/some/other/topic'),
            ]
        ),
    ])
