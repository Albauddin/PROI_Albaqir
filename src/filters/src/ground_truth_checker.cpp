#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class ModelStatesGroundTruthPublisher : public rclcpp::Node {
public:
    ModelStatesGroundTruthPublisher() : Node("model_states_ground_truth_publisher") {
        robot_name_ = this->declare_parameter<std::string>("robot_name", "burger"); 
        sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&ModelStatesGroundTruthPublisher::callback, this, std::placeholders::_1)
        );
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/ground_truth_odom", 10
        );
        RCLCPP_INFO(this->get_logger(), "Publishing ground truth for model: %s", robot_name_.c_str());
    }

private:
    void callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);
        if (it != msg->name.end()) {
            size_t idx = std::distance(msg->name.begin(), it);
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = now();
            odom_msg.header.frame_id = "map";  // Or "odom", if that's what you want
            odom_msg.child_frame_id = "base_footprint"; // Or your robot's base link
            odom_msg.pose.pose = msg->pose[idx];
            odom_msg.twist.twist = msg->twist[idx]; // Publishes true velocity from Gazebo
            // Optionally set covariance to zeros or small values
            for (int i = 0; i < 36; ++i) {
                odom_msg.pose.covariance[i] = 0.0;
                odom_msg.twist.covariance[i] = 0.0;
            }
            pub_->publish(odom_msg);
            RCLCPP_INFO(this->get_logger(), "Published ground truth odom for '%s': x=%.2f y=%.2f", robot_name_.c_str(), msg->pose[idx].position.x, msg->pose[idx].position.y);
        } else {
            RCLCPP_WARN(this->get_logger(), "Robot model '%s' not found in /gazebo/model_states!", robot_name_.c_str());
        }
    }


    std::string robot_name_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelStatesGroundTruthPublisher>());
    rclcpp::shutdown();
    return 0;
}
