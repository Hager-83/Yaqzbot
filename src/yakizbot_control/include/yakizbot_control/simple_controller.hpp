
#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>   // FIX: encoder_positions topic type
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SimpleController : public rclcpp::Node
{
public:
    explicit SimpleController(const std::string& name);

private:

    // ======================================================
    // FIX: callback now uses encoder array instead of JointState
    // ======================================================
    void encoderCallback(
        const std_msgs::msg::Float32MultiArray &msg);

    // ======================================================
    // Subscriber
    // FIX: subscribe directly to /encoder_positions
    // ======================================================
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr encoder_sub_;

    // ======================================================
    // Publishers
    // ======================================================
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // ======================================================
    // TF Broadcaster
    // ======================================================
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ======================================================
    // Robot parameters
    // ======================================================
    double wheel_radius_;
    double wheel_separation_;

    // ======================================================
    // Robot pose in odom frame
    // ======================================================
    double x_;
    double y_;
    double theta_;

    // ======================================================
    // Previous encoder values
    // Used to compute wheel delta motion
    // ======================================================
    double fl_prev_;
    double fr_prev_;
    double rl_prev_;
    double rr_prev_;

    // ======================================================
    // Previous timestamp
    // ======================================================
    rclcpp::Time prev_time_;
};

#endif // SIMPLE_CONTROLLER_HPP
