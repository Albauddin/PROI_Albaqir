#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Load paths
    pkg_my_project = get_package_share_directory('my_cool_project')
    model = LaunchConfiguration('model')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    # Param files
    costmap_common = os.path.join(pkg_my_project, 'param', f'costmap_common_params_burger.yaml')
    local_costmap = os.path.join(pkg_my_project, 'param', 'local_costmap_params.yaml')
    global_costmap = os.path.join(pkg_my_project, 'param', 'global_costmap_params.yaml')
    move_base_params = os.path.join(pkg_my_project, 'param', 'move_base_params.yaml')
    dwa_params = os.path.join(pkg_my_project, 'param', f'dwa_local_planner_params_burger.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='burger',
            description='model type [burger, waffle, waffle_pi]'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Command velocity topic'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='Odometry topic'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[move_base_params, dwa_params]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[dwa_params, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[move_base_params, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': os.path.join(pkg_my_project, 'maps', 'mymap.yaml')}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'controller_server',
                    'planner_server',
                    'bt_navigator'
                ]
            }]
        )
    ])
