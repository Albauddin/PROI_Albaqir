#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <eigen3/Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using std::placeholders::_1;
using std::placeholders::_2;

class FilterNode : public rclcpp::Node
{
public:
    FilterNode() : Node("filter_node")
    {
        odom_sub_.subscribe(this, "/odom");
        imu_sub_.subscribe(this, "/imu");

        sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>(odom_sub_, imu_sub_, 10);
        sync_->registerCallback(std::bind(&FilterNode::sensorCallback, this, _1, _2));

        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/prediction", 10);

        RCLCPP_INFO(this->get_logger(), "FilterNode initialized");
    }

private:
    void sensorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                        const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received synchronized sensor data");

        // TODO: Implement filtering logic here
    }

    // ROS 2 interfaces
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;

    Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(6, 1);
};
  
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
