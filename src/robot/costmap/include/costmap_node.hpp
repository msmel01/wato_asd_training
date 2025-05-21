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
  
      void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
      
      void publishCostmap();
      
      void initializeCostmap();
      
      void convertToGrid(float range, float angle, int &x_grid, int &y_grid);
      
      void markObstacle(int x_grid, int y_grid);
      
      void inflateObstacles();
      
      nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg_;
  
    private:
    robot::CostmapCore costmap_;
  
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
  
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

      std::vector<std::vector<int>> costmap;

      static constexpr double resolution_ = 0.1;
      // Value set to cells with an obstacle
      static constexpr int obstacle_cost_ = 100;
      // width and height
      int width_; 
      int height_;  
      // origin coordinates
      int origin_x_;
      int origin_y_;

      // Inflation radius and costs
      static constexpr float inflation_radius_ = 1.0f;
      static constexpr int inflation_radius_cells_ = static_cast<int>(inflation_radius_ / resolution_);
      static constexpr int max_inflation_cost_ = 100; // make the cost must higher to give higher weight to obstacles
  };
   
  #endif 