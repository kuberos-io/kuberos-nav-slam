# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml



ARGUMENTS = [
    # Namespace
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'),

    DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack'),
    
    # SLAM
    DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Whether run a SLAM'),
    
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'),
    
    DeclareLaunchArgument(
        'localization', default_value='amcl',
        description='Localization method'),
    
    DeclareLaunchArgument(
        'map_file_path',
        default_value=PathJoinSubstitution([get_package_share_directory('nav2_bringup_extended'),
                                            'maps', 'maze.yaml'])),
    
    DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([get_package_share_directory('nav2_bringup'), 
                                            'params', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes'),
    
    # Startup settings
    DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'),

    DeclareLaunchArgument(
        'use_composition', default_value='true',
        description='Whether to use composed bringup'),
    
    DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'),

    DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'),

    DeclareLaunchArgument(
        'launch_navigation', default_value='True',
        description='launch navigation component?'),
]


def generate_launch_description():
    
    # Packages
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_nav2_bringup_extended = get_package_share_directory('nav2_bringup_extended')
    nav2_launch_dir = os.path.join(pkg_nav2_bringup, 'launch')
    
    
    # Launch args
    arg_namespace = LaunchConfiguration('namespace')
    arg_use_namespace = LaunchConfiguration('use_namespace')
    
    arg_slam = LaunchConfiguration('slam')
    arg_use_sim_time = LaunchConfiguration('use_sim_time')
    arg_localization = LaunchConfiguration('localization')
    arg_map_file_path = LaunchConfiguration('map_file_path')
    arg_params_file = LaunchConfiguration('params_file')
    
    arg_autostart = LaunchConfiguration('autostart')
    arg_use_composition = LaunchConfiguration('use_composition')
    arg_use_respawn = LaunchConfiguration('use_respawn')
    arg_log_level = LaunchConfiguration('log_level')
    arg_launch_navigation = LaunchConfiguration('launch_navigation')
    
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': arg_use_sim_time,
        'yaml_filename': arg_map_file_path}

    configured_params = RewrittenYaml(
        source_file=arg_params_file,
        root_key=arg_namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(arg_use_namespace),
            namespace=arg_namespace),

        Node(
            condition=IfCondition(arg_use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': arg_autostart}],
            arguments=['--ros-args', '--log-level', arg_log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup_extended, 'launch', 'localization', 'slamtoolbox_launch.py')),
            condition=IfCondition(PythonExpression(['"', arg_localization, '" == "slamtoolbox"'])),
            launch_arguments={'namespace': arg_namespace,
                              'use_sim_time': arg_use_sim_time,
                              'autostart': arg_autostart,
                              'use_respawn': arg_use_respawn,
                              }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup_extended, 'launch', 'localization', 'rtabmap_launch.py')),
            condition=IfCondition(PythonExpression(['"', arg_localization, '" == "rtabmap"'])),
            launch_arguments={'namespace': arg_namespace,
                              'use_sim_time': arg_use_sim_time,
                              'autostart': arg_autostart,
                              'use_respawn': arg_use_respawn,
                              }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup_extended, 'launch', 'localization', 'amcl_launch.py')),
            condition=IfCondition(PythonExpression(['"', arg_localization, '" == "amcl"'])),
            launch_arguments={'namespace': arg_namespace,
                              'map': arg_map_file_path,
                              'use_sim_time': arg_use_sim_time,
                              'autostart': arg_autostart,
                              'params_file': arg_params_file,
                              'use_composition': arg_use_composition,
                              'use_respawn': arg_use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup_extended, 'launch', 'localization', 'groundtruth_sim_launch.py')),
            condition=IfCondition(PythonExpression(['"', arg_localization, '" == "groundtruth"'])),
            launch_arguments={'namespace': arg_namespace,
                              'map': arg_map_file_path,
                              'use_sim_time': arg_use_sim_time,
                              'autostart': arg_autostart,
                              'params_file': arg_params_file,
                              'use_composition': arg_use_composition,
                              'use_respawn': arg_use_respawn,
                              'container_name': 'nav2_container'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
            condition=IfCondition(arg_launch_navigation),
            launch_arguments={'namespace': arg_namespace,
                              'use_sim_time': arg_use_sim_time,
                              'autostart': arg_autostart,
                              'params_file': arg_params_file,
                              'use_composition': arg_use_composition,
                              'use_respawn': arg_use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
