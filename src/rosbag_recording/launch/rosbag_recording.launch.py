#!/usr/bin/env python3

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, GroupAction, OpaqueFunction, LogInfo, ExecuteProcess, SetLaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.some_substitutions_type import SomeSubstitutionsType
from typing import Text
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    # Execution mode
    DeclareLaunchArgument('mode',
                          default_value='simulation',
                          choices=['dataset', 'simulation'],
                          description='Execution mode'),
    
    # Nav to pose task setup
    DeclareLaunchArgument('start_pose',
                          default_value='0.0,0.0,0.0',
                          description='Start pose in navigation task'),
    DeclareLaunchArgument('use_sim_time',
                          default_value='true',
                          description='Whether to use simulation time from Ignition'),
    
    DeclareLaunchArgument('publish_world_to_map_tf', default_value='True',
                          description='Publish world to map'),

    # Nav2
    DeclareLaunchArgument('publish_pose_from_ign',
                          default_value='True',
                          description='Set "False" to skip publication of groundtruth pose'),

    DeclareLaunchArgument('localization_method',
                          default_value='amcl',
                          description='Localization choice'),
    
    # ROS bags
    DeclareLaunchArgument('experiment_name',
                          default_value='test',
                          description='Name of experiment'),

    DeclareLaunchArgument('record_rosbag',
                          default_value='False',
                          description='Set "True" to record ros bag'),

    DeclareLaunchArgument('record_input_rosbag',
                          default_value='False',
                          description='Set "True" to record ros bag with localization input data'),

    DeclareLaunchArgument('rosbag_topics',
                          default_value='/groundtruth_pose; /base_link_pose; /tf; /tf_static; /task_status; /diagnostics',
                          description='Override rosbag topics to record'),
    
    DeclareLaunchArgument('input_dataset',
                          default_value='/ws/datasets/dataset_realrobot',
                          description='dataset rosbag to playback'),
    
    DeclareLaunchArgument('input_dataset_playback_rate',
                          default_value='1.0',
                          description='rate of dataset rosbag playback'),

    # Scene (Ignition world)
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World. Choices: maze, warehouse, depot'),
]


def generate_launch_description():

    # Directories
    pkg_nav2_edge_eval =  get_package_share_directory('nav2_edge_eval')

    # Launch args
    arg_start_pose = LaunchConfiguration('start_pose')

    arg_use_sim_time = LaunchConfiguration('use_sim_time')

    arg_localization_method = LaunchConfiguration('localization_method')
    arg_publish_pose_from_ign = LaunchConfiguration('publish_pose_from_ign')

    arg_record_rosbag = LaunchConfiguration('record_rosbag')
    arg_record_input_rosbag = LaunchConfiguration('record_input_rosbag')
    arg_experiment_name = LaunchConfiguration('experiment_name')
    arg_rosbag_topics = LaunchConfiguration('rosbag_topics')

    arg_world = LaunchConfiguration('world')
    
    arg_input_dataset = LaunchConfiguration('input_dataset')
    arg_input_dataset_playback_rate = LaunchConfiguration('input_dataset_playback_rate')

    arg_publish_world_to_map_tf = LaunchConfiguration('publish_world_to_map_tf')

    def get_rosbag_launch(context, *args, **kwargs):
        """
        Get group action to record rosbags
        """
        experiment_name = arg_experiment_name.perform(context)
        topics_from_arg = arg_rosbag_topics.perform(context)
        record_input_rosbag = arg_record_input_rosbag.perform(context)

        # rosbag path
        localization_method = arg_localization_method.perform(context)
        path = f'rosbags/{str(experiment_name)}_{datetime.now().strftime("%Y%m%d%H%M%S")}'

        # topic list
        topic_list = [item.strip() for item in topics_from_arg.split(';')]
        if localization_method == 'amcl':
            topic_list.append('/amcl_pose')
            
        if record_input_rosbag in ['True', 'true', '1']:
            topic_list.append('/oakd/rgb/preview/camera_info')
            topic_list.append('/oakd/rgb/preview/depth')
            topic_list.append('/oakd/rgb/preview/image_raw')
            topic_list.append('/scan')
            topic_list.append('/tf')
            topic_list.append('/odom')

        # command
        cmd = ["ros2", "bag", "record", '-o', str(path), '--use-sim-time']
        cmd += topic_list

        # Group action
        rosbag_record_group_action = GroupAction(
                condition=IfCondition(arg_record_rosbag),
                actions=[
                    LogInfo(msg=[f"rosbag_record: ROS bag will be stored in {path}. Recording topics: {' '.join(topic_list)}"]),
                    LogInfo(msg=[f"rosbag_record: Executing: {' '.join(cmd)}"]),
                    ExecuteProcess(
                        name='rosbag_record',
                        cmd=cmd,
                        output='screen',
                        sigterm_timeout='10',
                        sigkill_timeout='8',
                        )
            ])

        return [rosbag_record_group_action]

    # Rosbag recording executor
    rosbag_record_func = OpaqueFunction(function=get_rosbag_launch)

    # Publisher: Turtlebot4 pose from Ignition
    ign_pose_pub_node = Node(
        condition=IfCondition(arg_publish_pose_from_ign),
        package='sim_data_publisher',
        executable='sim_data_publisher',
        name='sim_data_publisher',
        parameters=[{
            'use_sim_time': arg_use_sim_time,
            'world_name': arg_world,
            'publish_tf': LaunchConfiguration('publish_groundtruth_localization'),
            'robot_id': 'turtlebot4'}],
        output='screen',
        on_exit=Shutdown()
    )

    world_to_map_slam = Node(
        package='pose_to_static_tf',
        condition=IfCondition(PythonExpression([arg_publish_world_to_map_tf, ' and "', arg_localization_method, '" != "amcl"'])),
        executable='pose_to_static_tf',
        name='world_to_map_slam',
        parameters=[{'use_sim_time': arg_use_sim_time,
                     'pose_as_str': arg_start_pose,
                     'parent_frame_id': 'world',
                     'child_frame_id': 'map',
                     }],
    )

    world_to_map_amcl = Node(
        package='pose_to_static_tf',
        condition=IfCondition(PythonExpression([arg_publish_world_to_map_tf, ' and "', arg_localization_method, '" == "amcl"'])),
        executable='pose_to_static_tf',
        name='world_to_map_amcl',
        parameters=[{'use_sim_time': arg_use_sim_time,
                     'pose_as_str': '0,0,0',
                     'parent_frame_id': 'world',
                     'child_frame_id': 'map',
                     }],
    )

    tf_to_pose_node = Node(
        package='tf_to_pose',
        executable='tf_to_pose',
        name='tf_to_pose',
        parameters=[{'use_sim_time': arg_use_sim_time,
                     'parent_frame_id': 'world',
                     'child_frame_id': 'base_link',
                     }],
        remappings=[('tf_as_pose', 'base_link_pose')],
    )
    
    play_dataset = ExecuteProcess(
        name='rosbag_play',
        cmd=["ros2", "bag", "play", '--clock', '100', '--rate', arg_input_dataset_playback_rate, '--qos-profile-overrides-path',  PathJoinSubstitution([pkg_nav2_edge_eval, 'params', 'rosbag_play.yaml']), arg_input_dataset],
        output='screen',
        on_exit=Shutdown()
    )

    # Generate launch description
    ld = LaunchDescription(ARGUMENTS)
    
    ld.add_action(SetLaunchConfiguration(name='launch_navigation', condition=LaunchConfigurationEquals('mode', 'simulation'), value='True'))
    ld.add_action(SetLaunchConfiguration(name='launch_navigation', condition=LaunchConfigurationEquals('mode', 'dataset'), value='False'))
    ld.add_action(SetLaunchConfiguration(name='publish_groundtruth_localization', condition=LaunchConfigurationEquals('localization_method', 'groundtruth'), value='True'))
    ld.add_action(SetLaunchConfiguration(name='publish_groundtruth_localization', condition=LaunchConfigurationNotEquals('localization_method', 'groundtruth'), value='False'))
    
    ld.add_action(GroupAction(
        condition=LaunchConfigurationEquals('mode', 'dataset'),
        actions=[
            play_dataset
        ]
    ))
    
    ld.add_action(rosbag_record_func)
    ld.add_action(tf_to_pose_node)
    ld.add_action(world_to_map_amcl)
    ld.add_action(world_to_map_slam)
    return ld
