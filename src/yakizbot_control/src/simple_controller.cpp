#include "yakizbot_control/simple_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string& name)
: Node(name),
  x_(0.0), y_(0.0), theta_(0.0),
  fl_prev_(0.0), fr_prev_(0.0), rl_prev_(0.0), rr_prev_(0.0)
{
    declare_parameter("wheel_radius", 0.0425);
    declare_parameter("wheel_separation", 0.28);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    // Speed conversion matrix (kept for future use if needed)
    speed_conversion_ << wheel_radius_/2, wheel_radius_/2,
                         wheel_radius_/wheel_separation_,
                        -wheel_radius_/wheel_separation_;

    // ================================================
    // Wheel command publisher is disabled (Pico controls motors)
    // wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    //     "/simple_velocity_controller/commands", 10);
    // ================================================

    // Subscribe to joint states from JointStateBridge
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SimpleController::jointCallback, this, _1));

    // cmd_vel subscriber is disabled (Pico is handling motor control)
    // vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    //     "/cmd_vel", 10,
    //     std::bind(&SimpleController::velCallback, this, _1));

    // Odometry publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // TF Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    prev_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "SimpleController started in Odometry-only mode");
}

void SimpleController::velCallback(const geometry_msgs::msg::Twist &msg)
{
    (void)msg;  // suppress unused warning
}

void SimpleController::jointCallback(const sensor_msgs::msg::JointState &msg)
{
    if (msg.position.size() < 4)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Received JointState with less than 4 positions");
        return;
    }

    auto now = this->now();
    double dt = (now - prev_time_).seconds();
    if (dt <= 0.0001) return;

    double d_fl = msg.position[0] - fl_prev_;
    double d_fr = msg.position[1] - fr_prev_;
    double d_rl = msg.position[2] - rl_prev_;
    double d_rr = msg.position[3] - rr_prev_;

    fl_prev_ = msg.position[0];
    fr_prev_ = msg.position[1];
    rl_prev_ = msg.position[2];
    rr_prev_ = msg.position[3];

    // Skip update if no real movement (reduces drift when moving wheels manually)
    if (fabs(d_fl) < 0.001 && fabs(d_fr) < 0.001 && 
        fabs(d_rl) < 0.001 && fabs(d_rr) < 0.001)
    {
        prev_time_ = now;
        return;
    }

    prev_time_ = now;

    double d_s     = wheel_radius_ * (d_fl + d_fr + d_rl + d_rr) / 4.0;
    double d_theta = wheel_radius_ * 
                     ((d_fr + d_rr) - (d_fl + d_rl)) / (2.0 * wheel_separation_);

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // Publish Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Small covariance for better RViz behavior
    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.01;
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.01;
    odom.pose.covariance[28] = 0.01;
    odom.pose.covariance[35] = 0.01;

    odom_pub_->publish(odom);

    // Publish TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}