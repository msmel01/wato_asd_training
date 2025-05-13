#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Subscribe to costmap topic
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, _1));
  // Subscribe to odometry topic
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, _1));

  // Initialize publisher to global map topic
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer for updates
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  // Initialize global map
  global_map_.header.frame_id = "sim_world";

  global_map_.info.resolution = global_resolution; 
  global_map_.info.width = global_width_cells;
  global_map_.info.height = global_height_cells;
  
  global_map_.info.origin.position.x = -global_width_m / 2.0;
  global_map_.info.origin.position.y = -global_height_m / 2.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0; // identity quaternion

  global_map_.data.resize(global_width_cells * global_height_cells, -1); // all unknown

  // should_update_map_ = true;
}


void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the latest costmap
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}


void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // NEW CODE
  curr_x = x;
  curr_y = y;
  // RCLCPP_INFO(this->get_logger(), "Current robot odomoetry position x = %f, y = %f", curr_x, curr_y);
  // RCLCPP_INFO(this->get_logger(), "-------------");

  // Extract yaw
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  last_yaw = yaw;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= distance_threshold) {
      last_x = x;
      last_y = y;
      should_update_map_ = true;
      // RCLCPP_INFO(this->get_logger(), "Distance threshold travelled, must update map now");
  }
}


// Timer-based map update
void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
      // RCLCPP_INFO(this->get_logger(), "Calling integrateCostmap");
      integrateCostmap();

      global_map_.header.stamp = now();

      // // check that global map is actually updated
      // bool has_non_negative = false;
      // int count = 0;
      // for (size_t i = 0; i < global_map_.data.size(); ++i) {
      //     if (global_map_.data[i] > 0) {
      //         size_t x = i % global_map_.info.width;
      //         size_t y = i / global_map_.info.width;

      //         // RCLCPP_INFO(this->get_logger(), "Non -1 value found at (%zu, %zu): %d", x, y, global_map_.data[i]);
      //         has_non_negative = true;
      //         ++count;
      //     }
      // }
      
      // if (!has_non_negative) {
      //     RCLCPP_INFO(this->get_logger(), "All values in global_map_ are -1.");
      // }
      // RCLCPP_INFO(this->get_logger(), "Count of non -1 values is %d", count);

      map_pub_->publish(global_map_);

      should_update_map_ = false;
  }
}


// Linear fusion of costmaps
void MapMemoryNode::integrateCostmap() {
  const float res = latest_costmap_.info.resolution;
  const float origin_x_local = latest_costmap_.info.origin.position.x;
  const float origin_y_local = latest_costmap_.info.origin.position.y;

  float cos_yaw = std::cos(last_yaw);
  float sin_yaw = std::sin(last_yaw);


  for (size_t y_idx = 0; y_idx < latest_costmap_.info.height; ++y_idx) {
    for (size_t x_idx = 0; x_idx < latest_costmap_.info.width; ++x_idx) {
      // Cartesian coordinate in local frame
      float x_local_cell = origin_x_local + x_idx*res; // maybe add 0.5f to get center of cell cartesian coordinate
      float y_local_cell = origin_y_local + y_idx*res;

      // Cartesian coordinate in global frame using robot position and orientation
      float x_global_cell = last_x + (cos_yaw * x_local_cell - sin_yaw * y_local_cell);
      float y_global_cell = last_y + (sin_yaw * x_local_cell + cos_yaw * y_local_cell);

      // Check that cell is not out of bounds of global map
      // if (x_global_cell < 0 || y_global_cell < 0 || x_global_cell >= static_cast<int>(global_map_.info.width) || y_global_m >= static_cast<int>(global_map_.info.height)) {
      //   continue; // Skip if out of bounds
      // }

      // Corresponding global_map_ indices for cell's global coordinate
      int x_global_cell_idx = static_cast<int>((x_global_cell - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int y_global_cell_idx = static_cast<int>((y_global_cell - global_map_.info.origin.position.y) / global_map_.info.resolution);

      
      // Check that indices are not out of bounds
      if (x_global_cell_idx < 0 || y_global_cell_idx < 0 || x_global_cell_idx >= static_cast<int>(global_map_.info.width) || y_global_cell_idx >= static_cast<int>(global_map_.info.height)) {
        continue; // Skip if out of bounds
      }

      // Convert to 1D global_map_ index
      int global_idx = y_global_cell_idx * global_map_.info.width + x_global_cell_idx;
      int8_t& global_val = global_map_.data[global_idx];

      int local_idx = y_idx * latest_costmap_.info.width + x_idx;
      int8_t value = latest_costmap_.data[local_idx];
      
      // If a cell in the new costmap has a known value, overwrite its value into the global map
      if (global_val == -1 || value > global_val) {
        global_val = value;
        // RCLCPP_INFO(this->get_logger(), "value in global map updated");
      }
    }
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
