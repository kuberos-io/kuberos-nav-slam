#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, Shutdown, GroupAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    rtabmap_frame_id = LaunchConfiguration('rtabmap_frame_id')
    use_scan = LaunchConfiguration('use_scan')
    use_odom = LaunchConfiguration('use_odom')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmapviz = LaunchConfiguration('rtabmapviz')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    rgb_image_topic = LaunchConfiguration('rgb_image_topic')
    
    rtabmap_reg_strategy = LaunchConfiguration('rtabmap_reg_strategy')
    rtabmap_odom_strategy = LaunchConfiguration('rtabmap_odom_strategy')
    rtabmap_odom_f2m_maxsize = LaunchConfiguration('rtabmap_odom_f2m_maxsize')
    rtabmap_vis_maxfeatures = LaunchConfiguration('rtabmap_vis_maxfeatures')
    rtabmap_gftt_mindistance = LaunchConfiguration('rtabmap_gftt_mindistance')
    rtabmap_vis_cortype = LaunchConfiguration('rtabmap_vis_cortype')
    
    return LaunchDescription([
    DeclareLaunchArgument('namespace', default_value='', description='Namespace'),
    DeclareLaunchArgument('use_scan', default_value='True', description='whether to use the LIDAR for rtabmap'),
    DeclareLaunchArgument('use_odom', default_value='True', description='whether to use the odometry for rtabmap'),
    DeclareLaunchArgument('rtabmapviz', default_value='False', description='whether to launch rtabmap visualization GUI'),
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument('rtabmap_frame_id', default_value='base_link', description='id of the base frame used in rtabmap'),
    DeclareLaunchArgument('depth_image_topic', default_value='/oakd/rgb/preview/depth', description='topic for the depth camera image'),
    DeclareLaunchArgument('camera_info_topic', default_value='/oakd/rgb/preview/camera_info', description='topic for the camera info'),
    DeclareLaunchArgument('rgb_image_topic', default_value='/oakd/rgb/preview/image_raw', description='topic for the rgb camera image'),

    # RTabmap parameters
    DeclareLaunchArgument('rtabmap_reg_strategy', default_value="'1'", description='Reg/Strategy'),
    DeclareLaunchArgument('rtabmap_odom_strategy', default_value="'1'", description='Odom/Strategy'),
    DeclareLaunchArgument('rtabmap_odom_f2m_maxsize', default_value="'2000'", description='OdomF2M/MaxSize'),
    DeclareLaunchArgument('rtabmap_vis_maxfeatures', default_value="'1000'", description='Vis/MaxFeatures'),
    DeclareLaunchArgument('rtabmap_gftt_mindistance', default_value="'10'", description='GFTT/MinDistance'),
    DeclareLaunchArgument('rtabmap_vis_cortype', default_value="'1'", description='Vis/CorType'),

    
    GroupAction([
        PushRosNamespace(
            namespace=namespace),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id': rtabmap_frame_id,
                'use_sim_time': use_sim_time,
                'latch': True,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': use_scan,
                'subscribe_odom': use_odom,
                'Optimizer/GravitySigma': '0', # Disable imu constraints (we are already in 2D)
                'Reg/Force3DoF': '1', # 2D slam
                'RGBD/NeighborLinkRefining': '1', # odometry correction with scans
                'Reg/Strategy': '1', # 1 for lidar
                'Grid/FromDepth': 'false',
                'RGBD/ProximityPathMaxNeighbors': '10',
                # <<< Parameter Optimization
                'Odom/Strategy': rtabmap_odom_strategy,
                'OdomF2M/MaxSize': rtabmap_odom_f2m_maxsize,
                'Vis/CorType': rtabmap_vis_cortype,
                'Vis/MaxFeatures': rtabmap_vis_maxfeatures,
                'GFTT/MinDistance': rtabmap_gftt_mindistance,
                # Parameter Optimization >>>
                'qos_scan': 2,
                'queue_size': 10,
                'approx_sync': True}],
            remappings=[('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('rgb/image', rgb_image_topic),
                ('rgb/camera_info', camera_info_topic),
                ('depth/image', depth_image_topic)],
                # ('/scan', 'scan')],
            arguments=['-d']),

        Node(
            condition=IfCondition(rtabmapviz), 
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
                    'frame_id': rtabmap_frame_id,
                    'use_sim_time': use_sim_time,
                    'subscribe_depth': True,
                    'subscribe_scan': True,
                    'qos_scan': 2,
                    'qos_depth': 2,
                    'qos_rgb': 2,
                    'qos_imu': 2}],
            remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]),
        ]),
    ])
