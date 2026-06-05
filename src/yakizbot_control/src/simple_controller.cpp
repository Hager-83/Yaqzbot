#include "yakizbot_control/simple_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string& name)
: Node(name),
  x_(0.0), y_(0.0), theta_(0.0)
{
    declare_parameter("wheel_radius", 0.0425);
    declare_parameter("wheel_separation", 0.28);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    // ==========================================
    // Subscribe to joint_states from micro-ROS Pico
    // ==========================================
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SimpleController::jointStateCallback, this, _1));

    // Odometry publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    prev_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "SimpleController running (encoder-based odom)");
}


// ======================================================
// FIXED CALLBACK
// ======================================================
void SimpleController::jointStateCallback(
    const sensor_msgs::msg::JointState &msg)
{
    if (msg.velocity.size() < 4) return;

    auto now = this->now();
    double dt = (now - prev_time_).seconds();
    if (dt <= 0.0001) return;
    prev_time_ = now;

    double v_fl = msg.velocity[0];
    double v_rl = msg.velocity[1];
    double v_fr = - msg.velocity[2];   //<-----------------
    double v_rr = - msg.velocity[3];

    double v_left  = (v_fl + v_rl) / 2.0;
    double v_right = (v_fr + v_rr) / 2.0;

    double v_s     = wheel_radius_ * (v_left + v_right) / 2.0;
    double v_theta = wheel_radius_ * (v_right - v_left) / wheel_separation_;

    theta_ += v_theta * dt;
    x_     += v_s * cos(theta_) * dt;
    y_     += v_s * sin(theta_) * dt;


    // ==========================================
    // Publish Odometry
    // ==========================================
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    theta_ = atan2(sin(theta_), cos(theta_));
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v_s;
    odom.twist.twist.angular.z = v_theta;

    odom_pub_->publish(odom);

    // ==========================================
    // Publish TF (odom -> base_footprint)
    // ==========================================
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";   
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);

    /*
    RCLCPP_INFO(this->get_logger(),
        "vel: %f %f %f %f",
        msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]);  
    */
    
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

