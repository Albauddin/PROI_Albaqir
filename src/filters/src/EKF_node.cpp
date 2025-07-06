#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "filters/EKF.hpp"
#include <random>

class EKFNode : public rclcpp::Node {
public:
    EKFNode()
    : Node("ekf_node"), is_initialized_(false)
    {
        // Subscribe to /odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&EKFNode::odomCallback, this, std::placeholders::_1)
        );
        // Publisher for filtered pose
        ekf_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ekf_pose", 10
        );
    }

private:
    std::default_random_engine gen_;
    std::normal_distribution<double> noise_dist_{0.0, 0.05}; // mean=0, stddev=0.05 (5cm)

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // Yaw extraction from quaternion
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double theta = std::atan2(siny_cosp, cosy_cosp);

        // Velocity commands from odom
        double v = msg->twist.twist.linear.x;
        double omega = msg->twist.twist.angular.z;

        rclcpp::Time curr_time = msg->header.stamp;
        double dt = 0.05; // Default to 20 Hz

        double noisy_x = x + noise_dist_(gen_);
        double noisy_y = y + noise_dist_(gen_);
        double noisy_theta = theta + noise_dist_(gen_) * 0.1; // (smaller angular noise, optional)
        Eigen::Vector3d z(noisy_x, noisy_y, noisy_theta);

        //Eigen::Vector3d z(x, y, theta); // <-- Declare here
        Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 1.0; // Process noise
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 2.5; // Measurement noise

        if (!is_initialized_) {
            // Initial state and covariance
            Eigen::Vector3d x0(x, y, theta);
            Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * 0.001;
            ekf_.init(x0, P0);
            last_time_ = curr_time;
            is_initialized_ = true;
            return;
        }

        dt = (curr_time - last_time_).seconds();
        last_time_ = curr_time;

        // Prediction step
        ekf_.predict(v, omega, dt, Q);

        // Correction step
        ekf_.update(z, R);

        // Publish result
        publishEKFPose(msg->header.stamp);
    }


    void publishEKFPose(const rclcpp::Time& stamp)
    {
        auto state = ekf_.getState();
        auto cov = ekf_.getCovariance();

        geometry_msgs::msg::PoseWithCovarianceStamped out_msg;
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = "odom";

        out_msg.pose.pose.position.x = state(0);
        out_msg.pose.pose.position.y = state(1);
        out_msg.pose.pose.position.z = 0.0;

        // Convert theta to quaternion
        double theta = state(2);
        out_msg.pose.pose.orientation.w = std::cos(theta / 2.0);
        out_msg.pose.pose.orientation.x = 0.0;
        out_msg.pose.pose.orientation.y = 0.0;
        out_msg.pose.pose.orientation.z = std::sin(theta / 2.0);

        // Fill covariance (row-major)
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                out_msg.pose.covariance[i * 6 + j] = cov(i, j);

        ekf_pub_->publish(out_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pub_;
    EKF ekf_;
    rclcpp::Time last_time_;
    bool is_initialized_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
