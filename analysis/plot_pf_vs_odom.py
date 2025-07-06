import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
import csv
import pickle

SAVE_DIR = "/workspace/data/analysis"
CSV_PATH = os.path.join(SAVE_DIR, "pf_vs_odom.csv")
PNG_PATH = os.path.join(SAVE_DIR, "pf_vs_odom_particles.png")
CLOUD_PATH = os.path.join(SAVE_DIR, "particle_clouds.pkl")

def plot_covariance_ellipse(ax, mean, cov, n_std=2.0, **kwargs):
    if np.any(np.isnan(cov)) or np.any(np.isinf(cov)):
        return
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

class PFLogger(Node):
    def __init__(self):
        super().__init__('pf_logger')
        self.ekf_pose = None
        self.odom_pose = None
        self.ekf_data = []
        self.odom_data = []
        self.time_data = []
        self.ekf_covariances = []

        self.particle_clouds = []  # List of (time, cloud_arr)
        self.last_particle_time = None

        self.counter = 0

        self.create_subscription(PoseWithCovarianceStamped, '/pf_pose', self.ekf_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/particle_filter', self.particle_callback, 10)

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
                cov = np.array([
                    [self.ekf_pose.pose.covariance[0], self.ekf_pose.pose.covariance[1]],
                    [self.ekf_pose.pose.covariance[6], self.ekf_pose.pose.covariance[7]]
                ])
                self.ekf_data.append([ekf_x, ekf_y])
                self.odom_data.append([odom_x, odom_y])
                self.time_data.append(t)
                self.ekf_covariances.append(cov)
            self.counter += 1

    def particle_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        # Store cloud every 15 seconds
        if self.last_particle_time is None or t - self.last_particle_time >= 15.0:
            cloud_arr = []
            for pose in msg.poses:
                x = pose.position.x
                y = pose.position.y
                # NOTE: No weight in PoseArray—add here if available
                cloud_arr.append((x, y))
            self.particle_clouds.append((t, np.array(cloud_arr)))
            self.last_particle_time = t

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

    def save_clouds(self, filename=CLOUD_PATH):
        with open(filename, 'wb') as f:
            pickle.dump(self.particle_clouds, f)

def load_particle_clouds(filename=CLOUD_PATH):
    with open(filename, 'rb') as f:
        return pickle.load(f)

def plot_csv(csv_path=CSV_PATH, png_path=PNG_PATH, ekf_covariances=None, time_data=None, particle_clouds=None):
    import pandas as pd
    df = pd.read_csv(csv_path)
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(df['ekf_x'], df['ekf_y'], label='PF Pose', marker='o', markersize=3, linestyle='-')
    ax.plot(df['odom_x'], df['odom_y'], label='Odometry (Ground Truth)', marker='x', markersize=3, linestyle='-')

    # Covariance ellipses every 15s
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

    # Plot particle clouds every 15s
    if particle_clouds is not None:
        for idx, (t, cloud) in enumerate(particle_clouds):
            ax.scatter(cloud[:,0], cloud[:,1], color='gray', alpha=0.25, s=14, label='Particles' if idx==0 else "")
            # If you add weights, use: ax.scatter(cloud[:,0], cloud[:,1], c=weights, ...)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    ax.set_title('PF Pose vs Odometry 200 particles\n(Particles & Covariance every 15s)')
    ax.grid(True)
    ax.axis('equal')
    plt.tight_layout()
    plt.savefig(png_path, dpi=300)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PFLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Saving and plotting...')
        node.save_to_csv()
        node.save_clouds()
        particle_clouds = load_particle_clouds()
        plot_csv(
            ekf_covariances=node.ekf_covariances,
            time_data=node.time_data,
            particle_clouds=particle_clouds
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
