from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
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
        )
    ])