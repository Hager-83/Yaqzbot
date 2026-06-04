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

    // ==========================================
    // FIX: Use encoder_positions instead of joint_states
    // ==========================================
    encoder_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/encoder_positions", 10,                      // <-- CHANGED SOURCE
        std::bind(&SimpleController::encoderCallback, this, _1));

    // Odometry publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    prev_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "SimpleController running (encoder-based odom)");
}


// ======================================================
// FIXED CALLBACK: now uses raw encoder data
// ======================================================
void SimpleController::encoderCallback(const std_msgs::msg::Float32MultiArray &msg)
{
    if (msg.data.size() < 4)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid encoder data");
        return;
    }

    auto now = this->now();
    double dt = (now - prev_time_).seconds();
    if (dt <= 0.0001) return;

    // ==========================================
    // FIX: direct encoder delta (stable source)
    // ==========================================
    double d_fl = msg.data[0] - fl_prev_;
    double d_fr = msg.data[1] - fr_prev_;
    double d_rl = msg.data[2] - rl_prev_;
    double d_rr = msg.data[3] - rr_prev_;

    fl_prev_ = msg.data[0];
    fr_prev_ = msg.data[1];
    rl_prev_ = msg.data[2];
    rr_prev_ = msg.data[3];

    prev_time_ = now;

    // ==========================================
    // Differential drive odometry
    // ==========================================
    double d_s =
        wheel_radius_ * (d_fl + d_fr + d_rl + d_rr) / 4.0;

    double d_theta =
        wheel_radius_ *
        ((d_fr + d_rr) - (d_fl + d_rl)) /
        (2.0 * wheel_separation_);

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // ==========================================
    // Publish Odometry
    // ==========================================
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

    odom_pub_->publish(odom);

    // ==========================================
    // Publish TF (odom -> base_footprint)
    // ==========================================
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

    auto node =
        std::make_shared<SimpleController>("simple_controller");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}