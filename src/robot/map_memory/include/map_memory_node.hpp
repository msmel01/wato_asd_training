#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    // Global map parameters
    const float global_resolution = 0.1; // meters per cell
    const float global_width_m = 30.5;  // meters
    const float global_height_m = 30.5;
    const int global_width_cells = static_cast<int>(global_width_m / global_resolution); // array dimensions
    const int global_height_cells = static_cast<int>(global_height_m / global_resolution); // array dimensions

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    // NEW CODE
    double curr_x = 0.0;
    double curr_y = 0.0;
    double curr_yaw = 0.0;

    // Used to make sure we are updating every 1.5 m travelled
    double last_x = 0.0;
    double last_y = 0.0;
    double last_yaw = 0.0;  // in radians

    const double distance_threshold = 1.5;
    const double yaw_threshold = 0.05;

    // Flags
    bool costmap_updated_ = false;
    bool should_update_map_ = true;
    bool should_update_start_map_ = true;

    // Callback when new costmap is received
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // Callback when new robot position is received
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Update and publish new global costmap based on timer
    void updateMap();

    // Integrate latest costmap into global map
    // Would encapsulate into the MapMemoryCore with more time
    void integrateCostmap();
};

#endif 
