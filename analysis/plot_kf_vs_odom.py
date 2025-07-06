import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import csv

# === Set analysis directory and file names ===
SAVE_DIR = "/workspace/data/analysis"
CSV_PATH = os.path.join(SAVE_DIR, "kf_vs_odom.csv")
PNG_PATH = os.path.join(SAVE_DIR, "kf_vs_odom.png")

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')
        self.kf_pose = None
        self.odom_pose = None
        self.kf_data = []
        self.odom_data = []
        self.time_data = []
        self.counter = 0

        self.create_subscription(PoseWithCovarianceStamped, '/kf_pose', self.kf_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        os.makedirs(SAVE_DIR, exist_ok=True)

    def kf_callback(self, msg):
        self.kf_pose = msg
        self.save_if_ready()

    def odom_callback(self, msg):
        self.odom_pose = msg
        self.save_if_ready()

    def save_if_ready(self):
        if self.kf_pose is not None and self.odom_pose is not None:
            if self.counter % 5 == 0:
                t = self.get_clock().now().nanoseconds / 1e9
                kf_x = self.kf_pose.pose.pose.position.x
                kf_y = self.kf_pose.pose.pose.position.y
                odom_x = self.odom_pose.pose.pose.position.x
                odom_y = self.odom_pose.pose.pose.position.y

                self.kf_data.append([kf_x, kf_y])
                self.odom_data.append([odom_x, odom_y])
                self.time_data.append(t)
            self.counter += 1

    def save_to_csv(self, filename=CSV_PATH):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'kf_x', 'kf_y', 'odom_x', 'odom_y'])
            for i in range(len(self.kf_data)):
                writer.writerow([
                    self.time_data[i],
                    self.kf_data[i][0], self.kf_data[i][1],
                    self.odom_data[i][0], self.odom_data[i][1]
                ])

def plot_csv(filename=CSV_PATH, png_path=PNG_PATH):
    import pandas as pd
    df = pd.read_csv(filename)
    plt.figure(figsize=(8, 6))
    plt.plot(df['kf_x'], df['kf_y'], label='KF Pose', marker='o', markersize=3, linestyle='-')
    plt.plot(df['odom_x'], df['odom_y'], label='Odometry (Ground Truth)', marker='x', markersize=3, linestyle='-')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('KF Pose vs Odometry')
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(png_path, dpi=300)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Saving and plotting...')
        node.save_to_csv()
        plot_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
