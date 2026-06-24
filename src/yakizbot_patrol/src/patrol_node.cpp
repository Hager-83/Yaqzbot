#include "yakizbot_patrol/patrol_node.hpp"

using namespace std::chrono_literals;

namespace yakizbot_patrol
{

PatrolNode::PatrolNode() : Node("patrol_node"), current_wp_(0)
{
  declare_parameter("dwell_time", 5.0);
  declare_parameter("loop", true);
  declare_parameter("waypoint_names", std::vector<std::string>{});
  declare_parameter("waypoint_x",     std::vector<double>{});
  declare_parameter("waypoint_y",     std::vector<double>{});
  declare_parameter("waypoint_yaw",   std::vector<double>{});

  dwell_time_ = get_parameter("dwell_time").as_double();
  loop_       = get_parameter("loop").as_bool();

  auto names = get_parameter("waypoint_names").as_string_array();
  auto xs    = get_parameter("waypoint_x").as_double_array();
  auto ys    = get_parameter("waypoint_y").as_double_array();
  auto yaws  = get_parameter("waypoint_yaw").as_double_array();

  for (size_t i = 0; i < names.size(); ++i) {
    waypoints_.push_back({names[i], xs[i], ys[i], yaws[i]});
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints_.size());

  client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  timer_ = create_wall_timer(1s, [this]() {
    if (!client_->wait_for_action_server(0s)) {
      RCLCPP_INFO(get_logger(), "Waiting for navigate_to_pose...");
      return;
    }
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Action server ready, starting patrol");
    send_next_goal();
  });
}

void PatrolNode::send_next_goal()
{
  if (waypoints_.empty()) return;

  auto & wp = waypoints_[current_wp_];
  RCLCPP_INFO(get_logger(), "Going to: %s (%.2f, %.2f)", wp.name.c_str(), wp.x, wp.y);

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp    = now();
  goal.pose.pose.position.x = wp.x;
  goal.pose.pose.position.y = wp.y;

  tf2::Quaternion q;
  q.setRPY(0, 0, wp.yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  opts.result_callback = [this](const GoalHandle::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Reached %s, dwelling %.1fs",
        waypoints_[current_wp_].name.c_str(), dwell_time_);

      dwell_timer_ = create_wall_timer(
        std::chrono::duration<double>(dwell_time_),
        [this]() {
          dwell_timer_->cancel();
          advance_waypoint();
        });
    } else {
      RCLCPP_WARN(get_logger(), "Failed to reach %s, skipping",
        waypoints_[current_wp_].name.c_str());
      advance_waypoint();
    }
  };

  client_->async_send_goal(goal, opts);
}

void PatrolNode::advance_waypoint()
{
  current_wp_++;
  if (current_wp_ >= waypoints_.size()) {
    if (loop_) {
      RCLCPP_INFO(get_logger(), "Loop complete, restarting");
      current_wp_ = 0;
    } else {
      RCLCPP_INFO(get_logger(), "Patrol complete");
      return;
    }
  }
  send_next_goal();
}

}  // namespace yakizbot_patrol

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yakizbot_patrol::PatrolNode>());
  rclcpp::shutdown();
  return 0;
}