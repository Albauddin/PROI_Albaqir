#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import sin, cos
import time

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

POSES = [
    {"x": -0.807558, "y":  1.734820,  "yaw":  0.011524},
    {"x":  0.547704, "y":  0.713013,  "yaw": -1.145074},
    {"x": -0.587351, "y": -0.863849,  "yaw":  1.629065},
    {"x":  1.317420, "y": -1.679050,  "yaw":  2.200245},
]

def yaw_to_quaternion(yaw):
    return (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0))

class Nav2Demo(Node):
    def __init__(self):
        super().__init__('nav2_demo')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 NavigateToPose action server...')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(pose["x"])
        goal_msg.pose.pose.position.y = float(pose["y"])
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(pose["yaw"])
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Sending goal: ({pose['x']:.3f}, {pose['y']:.3f}, yaw={pose['yaw']:.3f})")
        send_goal_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return False

        get_result_future = goal_handle.get_result_async()
        self.get_logger().info('Waiting for robot to reach goal...')
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().warn(f"Goal finished with status {result.status}")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Demo()
    for pose in POSES:
        node.send_goal(pose)
        node.get_logger().info("Waiting 2 seconds before next goal...")
        time.sleep(2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
