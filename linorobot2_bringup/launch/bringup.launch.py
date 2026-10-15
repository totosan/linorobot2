# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            EmitEvent, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from linorobot2_base.domain_profile import resolve_ros_domain


def configure_ros_domain(context):
    profile = LaunchConfiguration('ros_profile').perform(context)
    requested_domain = LaunchConfiguration('ros_domain_id').perform(context)
    try:
        domain_id = resolve_ros_domain(
            profile,
            requested_domain,
            os.environ.get('ROS_DOMAIN_ID'),
            expected_profile='hardware',
        )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    return [SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=domain_id)]


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    default_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'default_robot.launch.py']
    )

    custom_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'custom_robot.launch.py']
    )

    extra_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'extra.launch.py']
    )

    lidar_monitor = Node(
        condition=IfCondition(LaunchConfiguration('lidar_required')),
        package='linorobot2_bringup',
        executable='lidar_monitor.py',
        name='lidar_monitor',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='ros_profile',
            default_value='hardware',
            description='Runtime profile; this bringup launch accepts hardware only'
        ),

        DeclareLaunchArgument(
            name='ros_domain_id',
            default_value='44',
            description='ROS domain required by the hardware profile'
        ),

        DeclareLaunchArgument(
            name='custom_robot', 
            default_value='false',
            description='Use custom robot'
        ),

        DeclareLaunchArgument(
            name='extra', 
            default_value='false',
            description='Launch extra launch file'
        ),
        
        DeclareLaunchArgument(
            name='webcam_enabled', 
            default_value='false',
            description='Enable the optional webcam and Contract-v1 off/mock adapter'
        ),

        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyUSB0',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='lidar_serial_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for the configured lidar'
        ),

        DeclareLaunchArgument(
            name='micro_ros_baudrate',
            default_value='921600',
            description='micro-ROS serial baudrate'
        ),

        DeclareLaunchArgument(
            name='micro_ros_transport',
            default_value='serial',
            description='micro-ROS transport'
        ),

        DeclareLaunchArgument(
            name='micro_ros_port',
            default_value='8888',
            description='micro-ROS udp/tcp port number'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),

        DeclareLaunchArgument(
            name='madgwick',
            default_value='false',
            description='Use madgwick to fuse imu and magnetometer'
        ),

        DeclareLaunchArgument(
            name='orientation_stddev',
            default_value='0.003162278',
            description='Madgwick orientation stddev'
        ),

        DeclareLaunchArgument(
            name='joy', 
            default_value='false',
            description='Use Joystick'
        ),

        DeclareLaunchArgument(
            name='micro_ros_enabled', 
            default_value='true',
            description='Enable micro-ROS'
        ),

        DeclareLaunchArgument(
            name='enable_command_timeout',
            default_value='false',
            description='Launch the supplemental ROS cmd_vel timeout node (disabled by default)'
        ),

        DeclareLaunchArgument(
            name='lidar_required',
            default_value='true',
            description='Shut down bringup if required lidar scans stop'
        ),

        OpaqueFunction(function=configure_ros_domain),

        RegisterEventHandler(
            OnProcessExit(
                target_action=lidar_monitor,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='Required lidar monitor exited'
                ))],
            )
        ),

        lidar_monitor,

        # This ROS-level watchdog is optional and is not a substitute for the
        # base firmware timeout, motor disable, or a physical emergency stop.
        Node(
            condition=IfCondition(LaunchConfiguration('enable_command_timeout')),
            package='linorobot2_base',
            executable='command_timeout.py',
            name='command_timeout',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("madgwick")),
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_node',
            output='screen',
            parameters=[{
                'orientation_stddev' : LaunchConfiguration('orientation_stddev'),
                'publish_tf' : False
            }]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_robot_launch_path),
            condition=UnlessCondition(LaunchConfiguration("custom_robot")),
            launch_arguments={
                'base_serial_port': LaunchConfiguration("base_serial_port"),
                'lidar_serial_port': LaunchConfiguration("lidar_serial_port"),
                'webcam_enabled': LaunchConfiguration("webcam_enabled"),
                'micro_ros_enabled': LaunchConfiguration("micro_ros_enabled"),
                'micro_ros_transport': LaunchConfiguration("micro_ros_transport"),
                'micro_ros_port': LaunchConfiguration("micro_ros_port"),
                'micro_ros_baudrate': LaunchConfiguration("micro_ros_baudrate")
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(extra_launch_path),
            condition=IfCondition(LaunchConfiguration("extra")),
            launch_arguments={
                'lidar_serial_port': LaunchConfiguration("lidar_serial_port")
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(custom_robot_launch_path),
            condition=IfCondition(LaunchConfiguration("custom_robot")),
            launch_arguments={
                'micro_ros_enabled': LaunchConfiguration("micro_ros_enabled")
            }.items()
        )
    ])
