#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_car_mapping = get_package_share_directory('car_mapping')
    
    # File paths
    urdf_file = os.path.join(pkg_car_mapping, 'urdf', 'car_robot.urdf')
    rviz_config_file = os.path.join(pkg_car_mapping, 'rviz', 'mapping.rviz')
    
    # Declare launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    # Robot description parameter - read URDF file content
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    
    robot_description = ParameterValue(
        robot_description_content,
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Static TF publishers for robot structure
    base_footprint_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )
    
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame_tf',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # Odom Bridge Node
    odom_bridge_node = Node(
        package='car_mapping',
        executable='odom_bridge.py',
        name='odom_bridge',
        output='screen'
    )
    
    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',#异步/async走走停停都能继续 ， 同步/sync 一口气采完
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'mode': 'mapping',
            'debug_logging': False,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_frame': 'map',
            'odom_frame': 'odom', 
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'minimum_time_interval': 0.3,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
            'scan_buffer_size': 50,  # 增加扫描缓冲区大小以容纳更多帧
            'scan_buffer_maximum_scan_distance': 10.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine': 0.45,
            'max_laser_range': 8.0,  # 设置为你的激光雷达最大范围
            'minimum_laser_range': 0.1,  # 设置为你的激光雷达最小范围
            'queue_size': 50,  # 增加消息队列大小以避免丢弃消息
            'process_rate': 5.0  # 降低处理频率，让CPU有时间处理消息
        }],
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Create maps directory
    create_maps_dir = ExecuteProcess(
        cmd=['mkdir', '-p', '/home/zzz/ros2_car/ros2/maps'],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_rviz_cmd,
        create_maps_dir,
        robot_state_publisher_node,
        base_footprint_to_base_tf,
        base_to_laser_tf,
        odom_bridge_node,
        slam_toolbox_node,
        rviz_node
    ])
