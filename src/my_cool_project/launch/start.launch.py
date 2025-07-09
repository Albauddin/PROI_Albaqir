import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# def generate_launch_description():
#     pkg_gazebo = os.path.join(
#         get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(pkg_gazebo)
#         )
#     ])

def generate_launch_description():
    pkg_gazebo = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    pkg_localization = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
    pkg_nav2_tb3 = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
    map_path = '/workspace/data/src/my_cool_project/maps/my_map.yaml'
    rviz_path = '/workspace/data/config/tb3_navigation2.rviz'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pkg_gazebo)
        ),
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(pkg_localization),
                    launch_arguments={'map': map_path}.items()
                )
            ]
        ),
        TimerAction(
            period=16.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(pkg_nav2_tb3),
                    launch_arguments={
                        'use_sim_time': 'True',
                        'map': map_path,
                        'rviz_config': rviz_path
                    }.items()
                )
            ]
        ),
    ])


# def generate_launch_description():
#     # Get paths for your other launch files
#     pkg_gazebo = os.path.join(
#         get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
#     pkg_nav2 = os.path.join(
#         get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
#     pkg_nav2_tb3 = os.path.join(
#         get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
#     pkg_filters = os.path.join(
#         get_package_share_directory('filters'), 'launch', 'filters.launch.py')
    
#     map_path = '/workspace/data/src/my_cool_project/maps/my_map.yaml'
#     rviz_path = '/workspace/data/config/tb3_navigation2.rviz'

#     def launch_sequence(context, *args, **kwargs):
#         print(f"map_path: '{map_path}', rviz_path: '{rviz_path}'")
#         print(f"pkg_gazebo: '{pkg_gazebo}'")
#         print(f"pkg_nav2: '{pkg_nav2}'")
#         print(f"pkg_nav2_tb3: '{pkg_nav2_tb3}'")
#         print(f"pkg_filters: '{pkg_filters}'")
#         print("STEP 1: Launching Gazebo...")
#         # 1. Start Gazebo
#         yield IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(pkg_gazebo)
#         )

        # # 2. Wait 2 seconds, then start localization
        # print("STEP 2: Waiting 2s, launching localization...")
        # yield TimerAction(
        #     period=6.0,
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(pkg_nav2),
        #             launch_arguments={'map': map_path}.items()
        #         )
        #     ]
        # )

        # # 3. Wait 2 more seconds, then start nav2 with RViz config
        # print("STEP 3: Waiting until 4s, launching navigation2 with RViz...")
        # yield TimerAction(
        #     period=12.0,  # 2 sec after last
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(pkg_nav2_tb3),
        #             launch_arguments={
        #                 'use_sim_time': 'True',
        #                 'map': map_path,
        #                 'rviz_config': rviz_path
        #             }.items()
        #         )
        #     ]
        # )

        # # 4. Wait 2 more seconds, then start your filter
        # print("STEP 4: Waiting until 6s, launching filters...")
        # yield TimerAction(
        #     period=18.0,
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(pkg_filters)
        #         )
        #     ]
        # )

        # # 5. Wait 2 more seconds, then run initial_pose_publisher
        # print("STEP 5: Waiting until 8s, launching initial_pose_publisher node...")
        # yield TimerAction(
        #     period=24.0,
        #     actions=[
        #         Node(
        #             package='my_cool_project',
        #             executable='initial_pose_publisher',
        #             output='screen'
        #         )
        #     ]
        # )

        # # 6. Wait 2 more seconds, then run your demo node
        # print("STEP 6: Waiting until 10s, launching run_demo node...")
        # yield TimerAction(
        #     period=30.0,
        #     actions=[
        #         Node(
        #             package='my_cool_project',
        #             executable='run_demo',
        #             output='screen'
        #         )
        #     ]
        # )

    # return LaunchDescription([
    #     OpaqueFunction(function=launch_sequence)
    # ])
