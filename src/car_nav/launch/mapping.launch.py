#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():


    pkg_car_nav = get_package_share_directory('car_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 使用 install 下的资源路径
    urdf_file = os.path.join(pkg_car_nav, 'urdf', 'car.urdf')
    rviz_config_file = os.path.join(pkg_car_nav, 'rviz', 'mapping.rviz')  # Fixed Frame 要设为 map
    map_yaml = os.path.join(pkg_car_nav, 'maps', 'room.yaml')             # 你的地图路径
    params_file = os.path.join(pkg_car_nav, 'config', 'nav2_params.yaml') # 你的Nav2参数路径
    map_pbstream = os.path.join(pkg_car_nav, 'maps-2', 'my_map.pbstream')   # cartographer保存的地图文件 - 绝对路径

    # 读取URDF
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    robot_description = ParameterValue(robot_description_content, value_type=str)



    # 4车轮状态发布节点
    wheel_node = Node(
        package='car_nav',
        executable='wheels_from_odom.py',  
        name='wheels_from_odom',
        output='screen'
    )
    # 二轴云台状态发布节点
    jq_node = Node(
        package='car_nav',
        executable='steering_from_odom.py',  
        name='steering_from_odom',
        output='screen'
    )

    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    {'tf_buffer_duration': 30.0}  # 增加 TF 缓存的时长，单位是秒
                    ]
    )

    

    # 加载静态地图
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_yaml,
        }]
    )

    # AMCL 定位节点（map->odom）
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        
    )
    # EKF 定位节点（odom->base_footprint）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[params_file],
        
        # 默认输出 /odometry/filtered，不建议改名
    )
    # 生命周期管理器（管理上面两个）
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl' ]}]
    )

    # slam_toolbox 节点 会自动发布map-odom和定位，不需要amcl和map_server
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False},
        ]
        
    )

    # 要包含config/carto_2d.lua文件（与yaml不通用）
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],   
        arguments=['-configuration_directory', '/home/zzz/ros2_car/ros2/src/car_nav/config',
                    '-configuration_basename', 'carto_localization.lua',  # 使用定位配置
                    '-load_state_filename', map_pbstream  ],# 加载保存的地图# 不是ros参数，必须用arguments（命令行参数）传递参数文件
        remappings=[
            ('scan', '/scan'),            # 如你的雷达话题不同，这里改
            # ('imu', '/imu'),            # 若使用 IMU，取消注释并改成你的话题
            ('odom', '/odometry/filtered'),  # 使用EKF融合后的里程计数据
        ]
    )
    # 可选：发布 OccupancyGrid（为Nav2提供地图数据，定位模式下降低频率）
    occ_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05,  # 地图分辨率
            'publish_period_sec': 1.0,  # 降低发布频率（定位模式下地图不变）
        }],
        remappings=[('map', '/map')]
    )

    
    # Controller Server - 局部路径跟踪
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Planner Server - 全局路径规划
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Behavior Server - 恢复行为
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )
    
        # BT Navigator - 行为树导航
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'default_bt_xml_filename': '/home/zzz/ros2_car/ros2/install/car_nav/share/car_nav/config/study_bt_navigator.xml',
            'default_nav_to_pose_bt_xml': '/home/zzz/ros2_car/ros2/install/car_nav/share/car_nav/config/study_bt_navigator.xml',
            'default_nav_through_poses_bt_xml': '/home/zzz/ros2_car/ros2/install/car_nav/share/car_nav/config/study_bt_navigator.xml'
        }, params_file]
    )
    
    # Waypoint Follower - 路径点跟随
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )
    
    # Navigation Lifecycle Manager - 管理导航节点
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server', 
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    # # 碰撞监测器
    # collision_monitor = Node(
    # package='nav2_collision_monitor',
    # executable='collision_monitor',
    # name='collision_monitor',
    # output='screen',
    # parameters=[params_file]
    # )


    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        
        jq_node,
        wheel_node,
        robot_state_publisher_node,
        map_server_node,
        amcl_node,
        ekf_node,
        lifecycle_manager_localization,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager_navigation,
        # slam_toolbox_node,
        # cartographer_node,
        # occ_grid_node,
        # collision_monitor,
        rviz_node  
    ])
