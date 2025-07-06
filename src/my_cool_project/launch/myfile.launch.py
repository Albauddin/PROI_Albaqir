from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_dir = os.path.join(
        get_package_share_directory('my_cool_project'),
        'maps',
        'my_map.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_dir}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen'
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': [
                            'map_server',
                            'amcl',
                            'planner_server',
                            'controller_server',
                            'bt_navigator'
                        ]}]
        )
    ])
