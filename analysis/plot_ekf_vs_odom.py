import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import csv
import numpy as np
from matplotlib.patches import Ellipse

SAVE_DIR = "/workspace/data/analysis"
CSV_PATH = os.path.join(SAVE_DIR, "ekf_vs_odom.csv")
PNG_PATH = os.path.join(SAVE_DIR, "ekf_vs_odom_cov.png")

def plot_covariance_ellipse(ax, mean, cov, n_std=2.0, **kwargs):
    # mean: (x, y)
    # cov: 2x2 covariance matrix
    if np.any(np.isnan(cov)) or np.any(np.isinf(cov)):
        return  # skip invalid covariances
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = eigvals.argsort()[::-1]
        eigvals, eigvecs = eigvals[order], eigvecs[:, order]
        angle = np.degrees(np.arctan2(*eigvecs[:, 0][::-1]))
        width, height = 2 * n_std * np.sqrt(np.abs(eigvals))
        ellip = Ellipse(xy=mean, width=width, height=height, angle=angle, **kwargs)
        ax.add_patch(ellip)
    except Exception as e:
        print(f"Covariance plotting failed at {mean}: {e}")

class PoseLogger(Node):
    def __init__(self):
        super().__init__('ekf_pose_logger')
        self.ekf_pose = None
        self.odom_pose = None
        self.ekf_data = []
        self.odom_data = []
        self.time_data = []
        self.ekf_covariances = []
        self.counter = 0

        self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.ekf_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        os.makedirs(SAVE_DIR, exist_ok=True)

    def ekf_callback(self, msg):
        self.ekf_pose = msg
        self.save_if_ready()

    def odom_callback(self, msg):
        self.odom_pose = msg
        self.save_if_ready()

    def save_if_ready(self):
        if self.ekf_pose is not None and self.odom_pose is not None:
            if self.counter % 5 == 0:
                t = self.get_clock().now().nanoseconds / 1e9
                ekf_x = self.ekf_pose.pose.pose.position.x
                ekf_y = self.ekf_pose.pose.pose.position.y
                odom_x = self.odom_pose.pose.pose.position.x
                odom_y = self.odom_pose.pose.pose.position.y
                # 2x2 covariance from 6x6 matrix (see ROS docs)
                cov = np.array([
                    [self.ekf_pose.pose.covariance[0], self.ekf_pose.pose.covariance[1]],
                    [self.ekf_pose.pose.covariance[6], self.ekf_pose.pose.covariance[7]]
                ])
                self.ekf_data.append([ekf_x, ekf_y])
                self.odom_data.append([odom_x, odom_y])
                self.time_data.append(t)
                self.ekf_covariances.append(cov)
            self.counter += 1

    def save_to_csv(self, filename=CSV_PATH):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'ekf_x', 'ekf_y', 'odom_x', 'odom_y'])
            for i in range(len(self.ekf_data)):
                writer.writerow([
                    self.time_data[i],
                    self.ekf_data[i][0], self.ekf_data[i][1],
                    self.odom_data[i][0], self.odom_data[i][1]
                ])

def plot_csv(csv_path=CSV_PATH, png_path=PNG_PATH, ekf_covariances=None, time_data=None):
    import pandas as pd
    df = pd.read_csv(csv_path)
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(df['ekf_x'], df['ekf_y'], label='EKF Pose', marker='o', markersize=3, linestyle='-')
    ax.plot(df['odom_x'], df['odom_y'], label='Odometry (Ground Truth)', marker='x', markersize=3, linestyle='-')
    # Draw covariance ellipses only at t=0, 5, 10, ...
    if ekf_covariances is not None and time_data is not None:
        shown_first = False
        last_time_shown = None
        for i in range(len(df)):
            t = time_data[i]
            if not shown_first:
                show = True
                shown_first = True
                last_time_shown = t
            elif t - last_time_shown >= 15.0:
                show = True
                last_time_shown = t
            else:
                show = False
            if show:
                cov = ekf_covariances[i] * 0.1
                mean = (df['ekf_x'][i], df['ekf_y'][i])
                plot_covariance_ellipse(ax, mean, cov, n_std=2, alpha=0.2, color='blue')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    ax.set_title('EKF Pose vs Odometry (Covariance every 15s)')
    ax.grid(True)
    ax.axis('equal')
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
        plot_csv(ekf_covariances=node.ekf_covariances, time_data=node.time_data)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
