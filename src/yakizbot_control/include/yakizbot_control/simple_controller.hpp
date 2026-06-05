#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SimpleController : public rclcpp::Node
{
public:
    explicit SimpleController(const std::string& name);

private:

    void jointStateCallback(
        const sensor_msgs::msg::JointState &msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double wheel_radius_;
    double wheel_separation_;

    double x_;
    double y_;
    double theta_;

    rclcpp::Time prev_time_;
};

#endif // SIMPLE_CONTROLLER_HPP