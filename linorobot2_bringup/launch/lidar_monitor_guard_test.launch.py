"""No-hardware launch fixture for the required-lidar failure path."""

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    monitor = Node(
        package='linorobot2_bringup',
        executable='lidar_monitor.py',
        name='lidar_monitor_guard_test',
        output='screen',
    )

    return LaunchDescription([
        RegisterEventHandler(
            OnProcessExit(
                target_action=monitor,
                on_exit=[
                    LogInfo(msg='LIDAR_MONITOR_EXIT_OBSERVED'),
                    EmitEvent(event=Shutdown(
                        reason='Required lidar monitor exited in guard test'
                    )),
                ],
            )
        ),
        monitor,
    ])
