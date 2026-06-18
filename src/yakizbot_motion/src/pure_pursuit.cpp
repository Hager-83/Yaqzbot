#include "yakizbot_motion/pure_pursuit.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>

namespace yakizbot_motion
{

void PurePursuit::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".look_ahead_distance", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_linear_velocity", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_slow_distance", rclcpp::ParameterValue(0.4));

  node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".min_linear_velocity", min_linear_velocity_);
  node->get_parameter(plugin_name_ + ".goal_slow_distance", goal_slow_distance_);

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pure_pursuit/carrot", 1);

  RCLCPP_INFO(logger_, "PurePursuit configured successfully - Yakizbot Graduation Project");
}

void PurePursuit::cleanup() { carrot_pub_.reset(); }
void PurePursuit::activate() { carrot_pub_->on_activate(); }
void PurePursuit::deactivate() { carrot_pub_->on_deactivate(); }

void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(logger_, "New plan received with %zu poses", path.poses.size());
}

void PurePursuit::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  current_speed_limit_ = speed_limit;
  use_speed_limit_percentage_ = percentage;
  RCLCPP_INFO(logger_, "Speed limit updated: %.2f %s", 
              speed_limit, percentage ? "(percentage)" : "(absolute)");
}

geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Empty global plan!");
    return cmd_vel;
  }

  if (!transformPlan(robot_pose.header.frame_id)) {
    RCLCPP_ERROR(logger_, "Failed to transform plan to robot frame");
    return cmd_vel;
  }

  // Collision checking using local costmap
  if (isCollisionAhead(robot_pose)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 500, "Collision ahead! Stopping.");
    return cmd_vel;  // zero velocity
  }

  auto carrot_pose = getCarrotPose(robot_pose);
  carrot_pub_->publish(carrot_pose);

  tf2::Transform robot_tf, carrot_tf, carrot_in_robot;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(carrot_pose.pose, carrot_tf);
  carrot_in_robot = robot_tf.inverse() * carrot_tf;

  double curvature = getCurvature(carrot_in_robot.getOrigin());

  // ==================== Speed Control ====================
  double linear_vel = getSpeedFromCurvature(curvature);

  // Goal slowing
  double dist_to_goal = std::hypot(
    global_plan_.poses.back().pose.position.x - robot_pose.pose.position.x,
    global_plan_.poses.back().pose.position.y - robot_pose.pose.position.y);

  if (dist_to_goal < goal_slow_distance_) {
    double slow_factor = dist_to_goal / goal_slow_distance_;
    linear_vel = std::max(min_linear_velocity_, linear_vel * slow_factor);
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Goal slowing active: %.2f m left", dist_to_goal);
  }

  // Apply speed limit
  if (use_speed_limit_percentage_) {
    linear_vel *= (current_speed_limit_ / 100.0);
  } else {
    linear_vel = std::min(linear_vel, current_speed_limit_);
  }

  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = curvature * max_angular_velocity_;

  RCLCPP_DEBUG(logger_, "Cmd: lin=%.3f, ang=%.3f, curv=%.3f", 
               linear_vel, cmd_vel.twist.angular.z, curvature);

  return cmd_vel;
}

// ==================== Helper Functions ====================

double PurePursuit::getSpeedFromCurvature(double curvature)
{
  // Reduce speed on sharp turns
  double abs_curv = std::abs(curvature);
  if (abs_curv > 1.5) return max_linear_velocity_ * 0.4;
  if (abs_curv > 0.8) return max_linear_velocity_ * 0.65;
  if (abs_curv > 0.4) return max_linear_velocity_ * 0.85;
  return max_linear_velocity_;
}

bool PurePursuit::isCollisionAhead(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (!costmap_) return false;

  int check_count = std::min(10, (int)global_plan_.poses.size());
  
  for (int i = 0; i < check_count; i++) {
    unsigned int mx, my;
    const auto & pose = global_plan_.poses[i];
    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) continue;
    unsigned char cost = costmap_->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::PoseStamped carrot = global_plan_.poses.back();
  for (auto it = global_plan_.poses.rbegin(); it != global_plan_.poses.rend(); ++it) {
    double dx = it->pose.position.x - robot_pose.pose.position.x;
    double dy = it->pose.position.y - robot_pose.pose.position.y;
    if (std::hypot(dx, dy) > look_ahead_distance_) {
      carrot = *it;
      break;
    }
  }
  return carrot;
}

double PurePursuit::getCurvature(const tf2::Vector3 & carrot_in_robot)
{
  double dx = carrot_in_robot.x();
  double dy = carrot_in_robot.y();
  double dist_sq = dx*dx + dy*dy;
  return (dist_sq > 0.001) ? (2.0 * dy / dist_sq) : 0.0;
}

bool PurePursuit::transformPlan(const std::string & frame)
{
  if (global_plan_.header.frame_id == frame) return true;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Transform failed: %s", ex.what());
    return false;
  }

  for (auto & pose : global_plan_.poses) {
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}

}  // namespace yakizbot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yakizbot_motion::PurePursuit, nav2_core::Controller)