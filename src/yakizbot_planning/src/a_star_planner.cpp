#include "yakizbot_planning/a_star_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <nav2_util/node_utils.hpp>



namespace yakizbot_planning
{

// ============================================================
// Helper: hash for GraphNode 
// ============================================================
struct NodeHash {
    std::size_t operator()(const GraphNode &n) const {
        return std::hash<int>()(n.x) ^ (std::hash<int>()(n.y) << 16);
    }
};

// ============================================================
// configure
// ============================================================
void AStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".tolerance", rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_iterations", rclcpp::ParameterValue(100000));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".cost_travel_multiplier", rclcpp::ParameterValue(2.0));

    node_->get_parameter(name_ + ".tolerance", tolerance_);
    node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
    node_->get_parameter(name_ + ".cost_travel_multiplier", cost_travel_multiplier_);

    RCLCPP_INFO(node_->get_logger(), "AStarPlanner configured: frame=%s", global_frame_.c_str());
}

void AStarPlanner::cleanup()  { RCLCPP_INFO(node_->get_logger(), "AStarPlanner cleanup"); }
void AStarPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "AStarPlanner activated"); }
void AStarPlanner::deactivate(){ RCLCPP_INFO(node_->get_logger(), "AStarPlanner deactivated"); }

// ============================================================
// Helpers
// ============================================================
bool AStarPlanner::inBounds(const GraphNode &n)
{
    return n.x >= 0 && n.y >= 0 &&
           n.x < static_cast<int>(costmap_->getSizeInCellsX()) &&
           n.y < static_cast<int>(costmap_->getSizeInCellsY());
}

double AStarPlanner::heuristic(const GraphNode &a, const GraphNode &b)
{
    // Diagonal (Chebyshev) heuristic — أحسن من Euclidean مع 8-connectivity
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
}

GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose &pose)
{
    unsigned int mx, my;
    costmap_->worldToMap(pose.position.x, pose.position.y, mx, my);
    return GraphNode(static_cast<int>(mx), static_cast<int>(my));
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode &node)
{
    geometry_msgs::msg::Pose pose;
    costmap_->mapToWorld(node.x, node.y, pose.position.x, pose.position.y);
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

// ============================================================
// createPlan — A* core
// ============================================================
nav_msgs::msg::Path AStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    std::function<bool()> cancel_checker)
{
    nav_msgs::msg::Path path;
    path.header.stamp  = node_->now();
    path.header.frame_id = global_frame_;

    GraphNode start_node = worldToGrid(start.pose);
    GraphNode goal_node  = worldToGrid(goal.pose);

    if (!inBounds(start_node) || !inBounds(goal_node)) {
        RCLCPP_WARN(node_->get_logger(), "Start or goal out of costmap bounds");
        return path;
    }

    // 8-directional neighbors
    const std::vector<std::pair<int,int>> dirs = {
        {1,0},{-1,0},{0,1},{0,-1},
        {1,1},{1,-1},{-1,1},{-1,-1}
    };
    const double DIAG_COST = std::sqrt(2.0);

    // Open set (min-heap)
    std::priority_queue<NodeRecord> open_set;
    // g-cost map
    std::unordered_map<GraphNode, double, NodeHash> g_cost;
    // came_from map
    std::unordered_map<GraphNode, GraphNode, NodeHash> came_from;

    g_cost[start_node] = 0.0;
    open_set.push({start_node, 0.0, heuristic(start_node, goal_node)});

    int iterations = 0;
    int max_iter = 100000;
    node_->get_parameter(name_ + ".max_iterations", max_iter);

    bool found = false;

    while (!open_set.empty() && iterations++ < max_iter)
    {
        if (cancel_checker()) {
            RCLCPP_INFO(node_->get_logger(), "Planning cancelled");
            return path;
        }

        NodeRecord current = open_set.top();
        open_set.pop();

        // Goal check (with tolerance)
        if (std::abs(current.node.x - goal_node.x) <= 1 &&
            std::abs(current.node.y - goal_node.y) <= 1)
        {
            goal_node = current.node;  // snap to last visited
            found = true;
            break;
        }

        for (const auto &dir : dirs)
        {
            GraphNode neighbor = current.node + dir;
            if (!inBounds(neighbor)) continue;

            unsigned char cost_val = costmap_->getCost(neighbor.x, neighbor.y);

            // Skip lethal unless allow_unknown handles it
            if (cost_val == nav2_costmap_2d::LETHAL_OBSTACLE) continue;
            if (cost_val == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) continue;

            bool diagonal = (dir.first != 0 && dir.second != 0);
            double move_cost = diagonal ? DIAG_COST : 1.0;

            // Scale by costmap value (0-252 → 1 to cost_travel_multiplier_)
            double cell_cost = 1.0 + cost_travel_multiplier_ *
                (static_cast<double>(cost_val) / nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

            double new_g = g_cost[current.node] + move_cost * cell_cost;

            if (g_cost.find(neighbor) == g_cost.end() || new_g < g_cost[neighbor])
            {
                g_cost[neighbor] = new_g;
                came_from[neighbor] = current.node;
                double f = new_g + heuristic(neighbor, goal_node);
                open_set.push({neighbor, new_g, f});
            }
        }
    }

    if (!found) {
        RCLCPP_WARN(node_->get_logger(), "A* failed to find a path");
        return path;
    }

    // Reconstruct path
    std::vector<GraphNode> raw_path;
    GraphNode cur = goal_node;
    while (!(cur == start_node)) {
        raw_path.push_back(cur);
        cur = came_from[cur];
    }
    raw_path.push_back(start_node);
    std::reverse(raw_path.begin(), raw_path.end());

    // Convert to PoseStamped
    for (const auto &node : raw_path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose   = gridToWorld(node);
        path.poses.push_back(ps);
    }

    RCLCPP_INFO(node_->get_logger(), "A* found path with %zu poses", path.poses.size());
    return path;
}

}  // namespace yakizbot_planning

PLUGINLIB_EXPORT_CLASS(yakizbot_planning::AStarPlanner, nav2_core::GlobalPlanner)