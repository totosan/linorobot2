from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for object visualization
        DeclareLaunchArgument(
            'enable_object_visualization',
            default_value='true',
            description='Enable object visualization markers in RViz'
        ),
        DeclareLaunchArgument(
            'marker_lifetime',
            default_value='2.0',
            description='Lifetime of markers in seconds'
        ),
        DeclareLaunchArgument(
            'show_object_points',
            default_value='true',
            description='Whether to show LiDAR points for detected objects'
        ),
        DeclareLaunchArgument(
            'show_object_labels',
            default_value='true',
            description='Whether to show object labels'
        ),
        DeclareLaunchArgument(
            'show_object_distance',
            default_value='true',
            description='Whether to show distance information'
        ),

        # Vision node (object detection and laser fusion)
        Node(
            package='linorobot2_vision',
            executable='vision_node',
            name='linorobot2_vision',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'image_topic': '/image_raw/compressed'},
                {'calib_file': PathJoinSubstitution([
                    FindPackageShare('linorobot2_vision'),
                    'data',
                    'calibration_result.txt'
                ])},
                {'config_file': PathJoinSubstitution([
                    FindPackageShare('linorobot2_vision'),
                    'config',
                    'camera_fusion_config.yaml'
                ])},
                {'laser_point_radius': 3},
                {'time_diff': 1.0}
            ],
            #arguments=['--ros-args', '--log-level', 'debug']
        ),

        # Object visualization node for RViz markers
        Node(
            package='linorobot2_vision',
            executable='object_visualization_node',
            name='object_visualization',
            output='screen',
            parameters=[{
                'marker_lifetime': LaunchConfiguration('marker_lifetime'),
                'text_size': 0.2,
                'point_size': 0.05,
                'show_points': LaunchConfiguration('show_object_points'),
                'show_labels': LaunchConfiguration('show_object_labels'),
                'show_distance': LaunchConfiguration('show_object_distance'),
            }],
            #arguments=['--ros-args', '--log-level', 'debug'],
            condition=IfCondition(LaunchConfiguration('enable_object_visualization'))
        )
    ])