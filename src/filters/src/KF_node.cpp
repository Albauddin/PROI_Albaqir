#include "filters/KF.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("kf_node"), first_(true) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&KalmanFilterNode::odomCallback, this, std::placeholders::_1)
        );
        kf_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_pose", 10);

        double dt = 4.9;
        kf_.configure(dt);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 1. Get measurement (no noise)
        double z_x = msg->pose.pose.position.x;
        double z_y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        Eigen::Vector3d z;
        z << z_x, z_y, yaw;

        // 2. Kalman filter initialization (only first callback)
        if (first_) {
            Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
            x0(0) = z(0);
            x0(1) = z(1);
            x0(2) = z(2);
            x0(3) = msg->twist.twist.linear.x;
            x0(4) = msg->twist.twist.linear.y;
            x0(5) = msg->twist.twist.angular.z;
            Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6,6);
            kf_.init(x0, P0);
            first_ = false;
        }

        // 3. Predict and update (Control input)
        Eigen::Vector3d u;
        u << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z;
        kf_.predict(u);
        kf_.update(z);

        Eigen::VectorXd x_hat = kf_.state();
        Eigen::MatrixXd P = kf_.covariance();

        // 4. Publish KF output as PoseWithCovarianceStamped
        geometry_msgs::msg::PoseWithCovarianceStamped kf_msg;
        kf_msg.header = msg->header;
        kf_msg.pose.pose.position.x = x_hat(0);
        kf_msg.pose.pose.position.y = x_hat(1);
        kf_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q_out;
        q_out.setRPY(0, 0, x_hat(2));
        kf_msg.pose.pose.orientation.x = q_out.x();
        kf_msg.pose.pose.orientation.y = q_out.y();
        kf_msg.pose.pose.orientation.z = q_out.z();
        kf_msg.pose.pose.orientation.w = q_out.w();
        for (int i = 0; i < 36; ++i)
            kf_msg.pose.covariance[i] = 0.0;
        kf_msg.pose.covariance[0] = P(0,0);    // x
        kf_msg.pose.covariance[7] = P(1,1);    // y
        kf_msg.pose.covariance[35] = P(2,2);   // theta

        kf_pub_->publish(kf_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr kf_pub_;
    KalmanFilter kf_;
    bool first_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
