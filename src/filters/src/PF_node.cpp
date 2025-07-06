#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"  // For publishing the particle cloud
#include <tf2/LinearMath/Quaternion.h>         // For tf2::Quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "filters/PF.hpp"
#include <tf2/utils.h>
#include <nav_msgs/msg/path.hpp>


class ParticleFilterNode : public rclcpp::Node {
public:
    ParticleFilterNode() : Node("particle_filter_node"), pf_(200) { // 50 particles, compact
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1)
        );
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pf_pose", 10
        );
        particle_cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/particle_filter", 10
        );
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pf_path", 10);
        // Init particles at (0,0,0)
        double std[] = {0.2, 0.2, 0.1};
        pf_.init(0.0, 0.0, 0.0, std);
    }

private:
    ParticleFilter pf_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::vector<geometry_msgs::msg::PoseStamped> pf_path_;


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Predict step
        double v = msg->twist.twist.linear.x;
        double w = msg->twist.twist.angular.z;
        double dt = 0.033; // 30 Hz
        double std_pos[] = {0.05, 0.05, 0.02};
        pf_.predict(v, w, dt, std_pos);

        // Measurement update
        auto pose = msg->pose.pose;
            tf2::Quaternion q(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            );
            double yaw = tf2::getYaw(q);

            Eigen::Vector3d measurement(
                pose.position.x,
                pose.position.y,
                yaw
            );
            double std_landmark[] = {0.2, 0.2, 0.05};
            pf_.update(measurement, std_landmark);

        // Publish best estimate
        auto est = pf_.getBestEstimate();
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.pose.position.x = est.x();
        pose_msg.pose.pose.position.y = est.y();
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation.x = 0.0;
        pose_msg.pose.pose.orientation.y = 0.0;
        pose_msg.pose.pose.orientation.w = cos(est.z() / 2.0);
        pose_msg.pose.pose.orientation.z = sin(est.z() / 2.0);
        pose_pub_->publish(pose_msg);

        // Publish particle cloud as PoseArray
        geometry_msgs::msg::PoseArray cloud_msg;
        cloud_msg.header = msg->header;
        for (const auto& p : pf_.getParticles()) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, p.theta);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            cloud_msg.poses.push_back(pose);
        }
        particle_cloud_pub_->publish(cloud_msg);

        // === Publish PF Trajectory as Path ===
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header = msg->header;
        stamped.pose = pose_msg.pose.pose;
        pf_path_.push_back(stamped);

        // Optional: Limit the history length for efficiency
        if (pf_path_.size() > 500)
            pf_path_.erase(pf_path_.begin());

        nav_msgs::msg::Path path_msg;
        path_msg.header = msg->header;
        path_msg.poses = pf_path_;
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
