#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.published = False

    def publish_pose(self):
        if self.published:
            rclpy.shutdown()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = -1.999919
        msg.pose.pose.position.y = -0.500001
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 4.934173183605763e-05
        msg.pose.pose.orientation.y = 0.002867817359352533
        msg.pose.pose.orientation.z = 6.272962636354709e-06
        msg.pose.pose.orientation.w = 0.9999958865663584
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]
        self.pub.publish(msg)
        self.get_logger().info('Initial pose published!')
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()