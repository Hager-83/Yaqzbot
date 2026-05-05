#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>

class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string& name);

private:
    void velCallback(const geometry_msgs::msg::Twist &msg);
    void jointCallback(const sensor_msgs::msg::JointState &msg);

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters
    double wheel_radius_;
    double wheel_separation_;

    // Kinematics
    Eigen::Matrix2d speed_conversion_;

    // Robot pose
    double x_, y_, theta_;

    // Previous wheel positions (for delta calculation)
    double fl_prev_, fr_prev_, rl_prev_, rr_prev_;

    // Time
    rclcpp::Time prev_time_;
};

#endif // SIMPLE_CONTROLLER_HPP