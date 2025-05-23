#include <chrono>
#include <memory>
#include <functional>

using std::placeholders::_1;

#include "costmap_node.hpp"
 

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  occup_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  scan_subs_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::subscribeMessage, this, _1));
}


void CostmapNode::publishGrid() {
  nav_msgs::msg::OccupancyGrid grid_msg;

  grid_msg.header.stamp = now();
  grid_msg.header.frame_id = "base_link"; // frame is robot-centered

  grid_msg.info.resolution = costmap_.getResolution();
  grid_msg.info.width = costmap_.getWidth(); // width of grid array
  grid_msg.info.height = costmap_.getHeight(); // height of grid array
  grid_msg.info.origin.position.x = -costmap_.getWidthWorld() / 2.0;
  grid_msg.info.origin.position.y = -costmap_.getHeightWorld() / 2.0;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);

  for (size_t y = 0; y < grid_msg.info.height; ++y) {
    for (size_t x = 0; x < grid_msg.info.width; ++x) {
      int value = costmap_.getCost(x, y);
      int8_t mapped_value = (value >= 100) ? 100 : static_cast<int8_t>(value); // clamp to max 100 and data type required for occupancy grid
      grid_msg.data[y * grid_msg.info.width + x] = mapped_value;
    }
  }

  // int max_val = *std::max_element(grid_msg.data.begin(), grid_msg.data.end());
  // RCLCPP_INFO(this->get_logger(), "Max cost in grid: %d", max_val); // check that there are non-zero values

  occup_grid_pub_->publish(grid_msg);
}


void CostmapNode::subscribeMessage(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // RCLCPP_INFO(this->get_logger(), "LaserScan frame_id: %s", scan->header.frame_id.c_str());

  costmap_.initializeGrid();
  // RCLCPP_INFO(this->get_logger(), "Finished initializeGrid");
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
        // Calculate grid coordinates
        int x_grid, y_grid;
        // RCLCPP_INFO(this->get_logger(), "Started convertToGrid");
        costmap_.convertToGrid(range, angle, x_grid, y_grid);
        // RCLCPP_INFO(this->get_logger(), "Finished convertToGrid");
        // RCLCPP_INFO(this->get_logger(), "Started markObstacle");
        costmap_.markObstacle(x_grid, y_grid);
        // RCLCPP_INFO(this->get_logger(), "Ended markObstacle");
    }
}

  costmap_.inflateObstacles();

  this->publishGrid();
  // RCLCPP_INFO(this->get_logger(), "New local costmap published");
  
  // RCLCPP_INFO(this->get_logger(), "Finished subscribeMessage");
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}