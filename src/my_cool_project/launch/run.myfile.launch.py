import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    filters_dir = get_package_share_directory('filters')
    my_cool_project_dir = get_package_share_directory('my_cool_project')

    # Path to your map yaml
    map_path = os.path.join(my_cool_project_dir, 'maps', 'my_map.yaml')

    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            )
        ),
        # Nav2 localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={'map': map_path}.items()
        ),
        # Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_nav2_dir, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'map': map_path
            }.items()
        ),
        # Your initial_pose_publisher node
        ExecuteProcess(
            cmd=['ros2', 'run', 'my_cool_project', 'initial_pose_publisher'],
            output='screen'
        ),
        # Kalman filter launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(filters_dir, 'launch', 'kalman.launch.py')
            )
        ),
    ])
