from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Kalman Filter Node
        Node(
            package='filters',
            executable='kf_node',
            name='kf_node',
            output='screen',
            remappings=[
                ('/odom', '/odom'),
                ('/kf_pose', '/kf_pose')
            ]
        ),
        # Extended Kalman Filter Node
        Node(
            package='filters',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            remappings=[
                ('/odom', '/odom'),
                ('/ekf_pose', '/ekf_pose')
            ]
        ),
        # Particle Filter Node
        Node(
            package='filters',
            executable='pf_node',
            name='pf_node',
            output='screen',
            remappings=[
                ('/odom', '/odom'),
                ('/pf_pose', '/pf_pose')
            ]
        ),
        # Ground Truth Checker Node
        Node(
            package='filters',
            executable='ground_truth_checker',
            name='ground_truth_checker',
            output='screen',
        ),
    ])
