import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    my_pkg = get_package_share_directory('my_cool_project')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    world_path = os.path.join(my_pkg, 'maps', '10by10_maze.world_1.xml')

    return LaunchDescription([
        # Gazebo server with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
            )
        ),

        # Publish URDF to robot_description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_pkg, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # ✅ Spawn TurtleBot3 with position
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_pkg, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '0.5',
                'y_pose': '0.5',
                'z_pose': '0.01',
                'entity_name': 'burger1'
            }.items()
        ),

        LogInfo(msg="✅ TurtleBot3 Burger spawn process launched.")
    ])
