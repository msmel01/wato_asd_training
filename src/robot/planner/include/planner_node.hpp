#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

  private:
    robot::PlannerCore planner_;

    // States
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    // Flags
    bool goal_received_ = false;

    // Data storage
    nav_msgs::msg::OccupancyGrid current_global_costmap_;
    geometry_msgs::msg::PointStamped goal_point_;
    geometry_msgs::msg::Pose robot_pose_;

    // Callback methods
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void newGoalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Other methods
    bool goalReached();
    void planPath();
    // AStar algorithm methods
    double calculateHeuristic(int x, int y, int g_x, int g_y);
    bool isObstacle(int x, int y);
    std::unordered_map<CellIndex, AStarNode, CellIndexHash> AStarAlgorithm();
};

#endif 
