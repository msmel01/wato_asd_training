#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    void publishGrid();

    void subscribeMessage(sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    robot::CostmapCore costmap_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occup_grid_pub_;

    // Subscribe to read
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subs_;
};
 
#endif 