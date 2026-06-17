#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace yakizbot_planning
{

struct GraphNode
{
    int x, y;
    GraphNode() : x(0), y(0) {}
    GraphNode(int px, int py) : x(px), y(py) {}
    bool operator==(const GraphNode &other) const
    {
        return x == other.x && y == other.y;
    }
    GraphNode operator+(const std::pair<int, int> &dir) const
    {
        return GraphNode(x + dir.first, y + dir.second);
    }
};

struct NodeRecord
{
    GraphNode node;
    double g;
    double f;

   bool operator<(const NodeRecord &other) const
{
    if (std::abs(f - other.f) < 1e-6)
    {
      return g < other.g;
    }
    return f > other.f;
}
};

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
    AStarPlanner() = default;
    ~AStarPlanner() = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> cancel_checker) override;

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D *costmap_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    std::string name_;
    std::string global_frame_;

    // Parameters
    double tolerance_ = 0.2;
    double cost_travel_multiplier_ = 2.0;
    bool allow_unknown_ = true;

    bool inBounds(const GraphNode &n);
    double heuristic(const GraphNode &a, const GraphNode &b);
    GraphNode worldToGrid(const geometry_msgs::msg::Pose &pose);
    geometry_msgs::msg::Pose gridToWorld(const GraphNode &node);
};

}  // namespace yakizbot_planning

#endif  // A_STAR_PLANNER_HPP