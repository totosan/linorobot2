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

from collections.abc import Mapping
import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


MAP_NAME='buro1' #change to the name of your own map here


def _merge_parameter_trees(common, profile):
    """Merge parameter mappings with profile values taking precedence."""
    if not isinstance(common, Mapping) or not isinstance(profile, Mapping):
        raise TypeError("Nav2 parameter roots must be mappings")

    merged = dict(common)
    for key, profile_value in profile.items():
        common_value = merged.get(key)
        if isinstance(common_value, Mapping) and isinstance(profile_value, Mapping):
            merged[key] = _merge_parameter_trees(common_value, profile_value)
        else:
            merged[key] = profile_value
    return merged


def _compose_parameter_file(common_path, profile_path, output_path=None):
    """Compose one standard ROS parameter file without loading sibling modules."""
    import yaml

    with open(common_path, encoding="utf-8") as stream:
        common = yaml.safe_load(stream)
    with open(profile_path, encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)

    merged = _merge_parameter_trees(common, profile)
    if output_path is None:
        descriptor, output_path = tempfile.mkstemp(
            prefix="linorobot2-nav2-", suffix=".yaml")
        stream = os.fdopen(descriptor, "w", encoding="utf-8")
    else:
        stream = open(output_path, "w", encoding="utf-8")
    with stream:
        yaml.safe_dump(merged, stream, sort_keys=False)
    return os.fspath(output_path)


def _launch_nav2(context):
    sim = LaunchConfiguration("sim").perform(context).lower() == "true"
    package_share = FindPackageShare('linorobot2_navigation')
    profile_path = (
        LaunchConfiguration('sim_params_file').perform(context)
        if sim else PathJoinSubstitution(
            [package_share, 'config', 'navigation.yaml']).perform(context)
    )
    common_path = LaunchConfiguration('common_params_file').perform(context)
    params_path = _compose_parameter_file(common_path, profile_path)

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'map': LaunchConfiguration("map"),
            'use_sim_time': LaunchConfiguration("sim"),
            'params_file': params_path
        }.items()
    )]


def generate_launch_description():
    depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_sim_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'navigation_sim.yaml']
    )
    nav2_common_config_path = os.path.join(
        get_package_share_directory('linorobot2_navigation'),
        'config', 'navigation_common.yaml')


    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='sim_params_file',
            default_value=nav2_sim_config_path,
            description='Simulation-specific Nav2 parameters; override for Fortress profiles'
        ),

        DeclareLaunchArgument(
            name='common_params_file',
            default_value=nav2_common_config_path,
            description='Shared planner/costmap parameters merged below the selected profile'
        ),

        OpaqueFunction(function=_launch_nav2),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])