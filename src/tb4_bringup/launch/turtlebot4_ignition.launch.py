# Copyright 2023 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


ARGUMENTS = [
    
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    
    # Ignition setup
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    
    DeclareLaunchArgument('world_var', default_value='',
                          description='World variation - select the randomized world model (sdf)'),
    
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    
    DeclareLaunchArgument('launch_simulation', default_value='True',
                          description='Set "false" to skip simulation startup.'),
    
    DeclareLaunchArgument('headless', default_value='True',
                          description='Start Igniton GUI or not'),
    
    DeclareLaunchArgument('publish_pose_from_ign',
                          default_value='False',
                          description='Set "False" to skip publication of groundtruth pose'),
    
]

# Inital robot pose in the world
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, 
                                           default_value='0.0',
                                           description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    
    # Directories
    pkg_tb4_bringup = get_package_share_directory('tb4_bringup')

    # Launch args
    arg_namespace = LaunchConfiguration('namespace')
    arg_world = LaunchConfiguration('world')
    arg_world_var = LaunchConfiguration('world_var')
    
    arg_publish_pose_from_ign = LaunchConfiguration('publish_pose_from_ign')
    
    arg_model = LaunchConfiguration('model')
    arg_headless = LaunchConfiguration('headless')
    arg_launch_simulation = LaunchConfiguration('launch_simulation')
    
    # Included launches
    ignition_launch = PathJoinSubstitution(
        [pkg_tb4_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_tb4_bringup, 'launch', 'turtlebot4_spawn.launch.py'])


    # Launch Ignition
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        condition=IfCondition(arg_launch_simulation),        
        launch_arguments=[
            ('world', arg_world),
            ('world_var', arg_world_var),
            ('headless', arg_headless),
        ]
    )

    # Launch to spawn a robot in Ignition
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', arg_namespace),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    ign_pose_pub_node = Node(
        condition=IfCondition(arg_publish_pose_from_ign),
        package='sim_data_publisher',
        executable='sim_data_publisher',
        name='sim_data_publisher',
        parameters=[{
            'use_sim_time': True,
            'world_name': arg_world,
            # 'publish_tf': LaunchConfiguration('publish_groundtruth_localization'),
            'publish_tf': False, #  LaunchConfiguration('publish_groundtruth_localization'),
            'robot_id': 'turtlebot4'}],
        output='screen',
        on_exit=Shutdown()
    )
    
    # Create launch description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    ld.add_action(ign_pose_pub_node)
    return ld
