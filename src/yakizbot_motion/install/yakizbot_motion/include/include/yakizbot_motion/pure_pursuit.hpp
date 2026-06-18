#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace yakizbot_motion
{
class PurePursuit : public nav2_core::Controller
{
public:
  PurePursuit() = default;
  ~PurePursuit() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  geometry_msgs::msg::PoseStamped getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose);
  bool transformPlan(const std::string & frame);
  double getCurvature(const tf2::Vector3 & carrot_in_robot);
  double getSpeedFromCurvature(double curvature);
  bool isCollisionAhead(const geometry_msgs::msg::PoseStamped & robot_pose);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> carrot_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;

  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuit")};
  rclcpp::Clock::SharedPtr clock_;

  double look_ahead_distance_ = 0.6;
  double max_linear_velocity_ = 0.35;
  double max_angular_velocity_ = 1.2;
  double min_linear_velocity_ = 0.1;        
  double goal_slow_distance_ = 0.4;         

  double current_speed_limit_ = 1.0;        
  bool use_speed_limit_percentage_ = false;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace yakizbot_motion

#endif  // PURE_PURSUIT_HPP