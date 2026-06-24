#ifndef PATROL_NODE_HPP
#define PATROL_NODE_HPP

#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace yakizbot_patrol
{

struct Waypoint {
  std::string name;
  double x, y, yaw;
};

class PatrolNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit PatrolNode();

private:
  void send_next_goal();
  void advance_waypoint();

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr dwell_timer_;

  std::vector<Waypoint> waypoints_;
  size_t current_wp_;
  double dwell_time_;
  bool loop_;
};

}  // namespace yakizbot_patrol

#endif  // PATROL_NODE_HPP