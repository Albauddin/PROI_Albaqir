from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    example_pkg = FindPackageShare("example_package")
    turtlebot3_desc = FindPackageShare("turtlebot3_description")

    urdf_file = PathJoinSubstitution([
        turtlebot3_desc,
        "urdf",
        "turtlebot3_burger_for_autorace.urdf.xacro"
    ])

    world_file = PathJoinSubstitution([
        example_pkg,
        "world",
        "playground.world"
    ])

    rviz_config_file = PathJoinSubstitution([
        example_pkg,
        "config",
        "config.rviz"
    ])

    return LaunchDescription([
        # Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
            ]),
            launch_arguments={
                "world": world_file,
                "gui": "true",
                "use_sim_time": "true",
                "paused": "false"
            }.items()
        ),

        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "use_sim_time": True,
                "robot_description": Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    urdf_file
                ])
            }]
        ),

        # Spawn the model in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_urdf",
            arguments=[
                "-entity", "turtlebot3_burger",
                "-topic", "robot_description",
                "-x", "0", "-y", "0", "-z", "0", "-Y", "0"
            ],
            output="screen"
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),

        Node(
            package="turtlebot3_teleop",
            executable="teleop_keyboard",
            name="teleop_keyboard",
            prefix="xterm -e",
            output="screen"
        )
    ])
