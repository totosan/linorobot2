# fortress.launch.py - Startskript für Ignition Fortress Simulation (ROS 2 Humble / Fortress)
# Referenz: docs/DEVELOPMENT_PLAN.md (Phasen 1-3)

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, Command,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from linorobot2_base.domain_profile import resolve_ros_domain


def configure_ros_domain(context):
    profile = LaunchConfiguration('ros_profile').perform(context)
    requested_domain = LaunchConfiguration('ros_domain_id').perform(context)
    try:
        domain_id = resolve_ros_domain(
            profile,
            requested_domain,
            os.environ.get('ROS_DOMAIN_ID'),
            expected_profile='sim',
        )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    return [SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=domain_id)]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "fortress_test.sdf"]
    )

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf", "robots", "2wd_fortress.urdf.xacro"]
    )

    bridge_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "config", "fortress_bridge.yaml"]
    )

    bridge_camera_fault_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "config", "fortress_bridge_camera_fault.yaml"]
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "config", "fortress_ekf.yaml"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "rviz", "fortress.rviz"]
    )

    vision_overlay_launch_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_vision"), "launch", "vision_overlay.launch.py"]
    )

    # Separate processes avoid the combined CLI's GUI/world-selection handshake
    # and allow server failures to terminate the complete launch deterministically.
    server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', LaunchConfiguration('world'), '-v', '3'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='ros_profile',
            default_value='sim',
            description='Runtime profile; this simulator launch accepts sim only'
        ),

        DeclareLaunchArgument(
            name='ros_domain_id',
            default_value='42',
            description='ROS domain required by the sim profile'
        ),

        OpaqueFunction(function=configure_ros_domain),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Simulationszeit verwenden'
        ),

        DeclareLaunchArgument(
            name='world',
            default_value=world_path,
            description='Ignition Fortress World Datei (.sdf)'
        ),

        DeclareLaunchArgument(
            name='spawn_x',
            default_value='0.0',
            description='Spawn-Position X'
        ),

        DeclareLaunchArgument(
            name='spawn_y',
            default_value='0.0',
            description='Spawn-Position Y'
        ),

        DeclareLaunchArgument(
            name='spawn_z',
            default_value='0.1',
            description='Spawn-Position Z'
        ),

        DeclareLaunchArgument(
            name='spawn_yaw',
            default_value='0.0',
            description='Spawn-Ausrichtung Yaw'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='RViz2 zusammen mit dem Simulator starten'
        ),

        DeclareLaunchArgument(
            name='rviz_config',
            default_value=rviz_config_path,
            description='Pfad zur RViz-Konfigurationsdatei (.rviz)'
        ),

        DeclareLaunchArgument(
            name='vision_view',
            default_value='false',
            description='Vision-Kameraoverlay, Fusion und eigenes RViz-Fenster starten (Mock)'
        ),

        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Gazebo-GUI starten; Server und Sensoren laufen auch ohne GUI'
        ),

        DeclareLaunchArgument(
            name='camera_fault_test',
            default_value='false',
            description='G3 test only: gate camera images for ROS topic outage injection'
        ),

        DeclareLaunchArgument(
            name='lidar_noise_stddev', default_value='0.0',
            description='Gaussian lidar range noise standard deviation in meters'
        ),
        DeclareLaunchArgument(
            name='imu_gyro_noise_stddev', default_value='0.0',
            description='Gaussian IMU angular-velocity noise standard deviation in rad/s'
        ),
        DeclareLaunchArgument(
            name='imu_accel_noise_stddev', default_value='0.0',
            description='Gaussian IMU linear-acceleration noise standard deviation in m/s^2'
        ),
        DeclareLaunchArgument(
            name='wheel_mu', default_value='1.0',
            description='Fortress wheel friction coefficient in both ODE directions'
        ),
        DeclareLaunchArgument(
            name='include_caster', default_value='true',
            description='G3 test only: include the front caster contact'
        ),
        DeclareLaunchArgument(
            name='caster_mu', default_value='0.01',
            description='G3 test only: caster friction coefficient'
        ),

        # 1. Server und GUI unabhängig starten; kein gemeinsamer Fork-Wrapper.
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Fortress-Server beendet'))]
        )),
        server,
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '-v', '3'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),

        # 2. Robot State Publisher mit Fortress 2WD URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ', urdf_path,
                    ' lidar_noise_stddev:=', LaunchConfiguration('lidar_noise_stddev'),
                    ' imu_gyro_noise_stddev:=', LaunchConfiguration('imu_gyro_noise_stddev'),
                    ' imu_accel_noise_stddev:=', LaunchConfiguration('imu_accel_noise_stddev'),
                    ' wheel_mu:=', LaunchConfiguration('wheel_mu'),
                    ' include_caster:=', LaunchConfiguration('include_caster'),
                    ' caster_mu:=', LaunchConfiguration('caster_mu')
                ])
            }]
        ),

        # 3. Roboter in Fortress spawnen
        # Anlaufverzögerung, kein Readiness-Nachweis: der Spawner wartet selbst
        # auf Weltnamen; der Smoke-Test prüft zusätzlich den echten World-Service.
        TimerAction(
            period=5.0,
            actions=[Node(
                package='ros_ign_gazebo',
                executable='create',
                name='urdf_spawner',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'linorobot2_2wd',
                    '-x', LaunchConfiguration('spawn_x'),
                    '-y', LaunchConfiguration('spawn_y'),
                    '-z', LaunchConfiguration('spawn_z'),
                    '-Y', LaunchConfiguration('spawn_yaw'),
                ]
            )]
        ),

        # 4. ROS-Ignition Parameter Bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config_path}],
            condition=UnlessCondition(LaunchConfiguration('camera_fault_test'))
        ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            parameters=[{'config_file': bridge_camera_fault_config_path}],
            condition=IfCondition(LaunchConfiguration('camera_fault_test'))
        ),
        ExecuteProcess(
            cmd=['python3', PathJoinSubstitution([
                FindPackageShare('linorobot2_gazebo'), 'scripts', 'camera_fault_proxy.py'])],
            name='g3_camera_fault_proxy',
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera_fault_test'))
        ),

        # 5. EKF Node für odom -> base_footprint TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "/odom")]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('rviz'), "' == 'true' and '",
                LaunchConfiguration('vision_view'), "' == 'false'"
            ]))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_overlay_launch_path),
            launch_arguments={'backend': 'mock'}.items(),
            condition=IfCondition(LaunchConfiguration('vision_view'))
        ),

        # 6. Sicherheits-Watchdog: Command Timeout
        Node(
            package='linorobot2_base',
            executable='command_timeout.py',
            name='command_timeout',
            # Wall-Clock-Watchdog: soll auch bei pausierter Simulationszeit ticken.
            parameters=[{'use_sim_time': False}]
        )
    ])
