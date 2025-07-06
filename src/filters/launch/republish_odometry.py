# republish_odometry.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_repub')
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom_slow', 10)
        self.last_msg = None
        self.timer = self.create_timer(0.5, self.publish_slow)  # 2 Hz

    def cb(self, msg):
        self.last_msg = msg

    def publish_slow(self):
        if self.last_msg:
            self.pub.publish(self.last_msg)

rclpy.init()
node = OdomRepublisher()
rclpy.spin(node)
