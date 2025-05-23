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
  // RCLCPP_INFO(this->get_logger(), "Costmap received");
  latest_costmap_ = *msg;
  // costmap_updated_ = true;

  // RCLCPP_INFO(this->get_logger(), "latest_costmap_ info: width=%d, height=%d, resolution=%f", latest_costmap_.info.width, latest_costmap_.info.height, latest_costmap_.info.resolution);
  // bool has_non_zero_in_callback = false;
  // for (size_t i = 0; i < latest_costmap_.data.size(); ++i) {
  //     if (latest_costmap_.data[i] != 0 && latest_costmap_.data[i] != -1) { // Check for occupied or unknown cells
  //         has_non_zero_in_callback = true;
  //         break;
  //     }
  // }
  // RCLCPP_INFO(this->get_logger(), "latest_costmap_.data has non-zero values in callback: %d", has_non_zero_in_callback);

  bool has_meaningful_data = false;
  // Iterate through a small sample or the whole data to check for known values (not -1 or 0)
  // Or, if 0 means 'free' space you expect, check for 1-100 (occupied)
  // Assuming 0 is free, -1 is unknown, 1-100 is occupied.
  for (size_t i = 0; i < latest_costmap_.data.size(); ++i) {
      if (latest_costmap_.data[i] > 0) { // Check for occupied cells
          has_meaningful_data = true;
          break;
      }
  }

  if (has_meaningful_data) {
      costmap_updated_ = true;
      // RCLCPP_INFO(this->get_logger(), "costmap_updated_ set to true (meaningful data detected).");
  } else {
      costmap_updated_ = false; // Or just don't set it if it was already false
      // RCLCPP_INFO(this->get_logger(), "Costmap received, but no meaningful (occupied) data yet.");
  }

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

  curr_yaw = yaw;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  double delta_yaw = std::abs(last_yaw - curr_yaw);
  // RCLCPP_INFO(this->get_logger(), "last_x, last_y, last_yaw are %f, %f, %f and distance travelled is %f", last_x, last_y, last_yaw, distance);
  // RCLCPP_INFO(this->get_logger(), "x, y, yaw are %f, %f, %f and delta_yaw is %f", x, y, yaw, delta_yaw);
  // RCLCPP_INFO(this->get_logger(), "-------");

  if (distance >= distance_threshold) {
    last_x = x;
    last_y = y;
    last_yaw = yaw;
    // if (delta_yaw <= yaw_threshold) {
    //   should_update_map_ = true;
    // }
    should_update_map_ = true;
  }
  // if (delta_yaw >= yaw_threshold) {
  //   last_yaw = curr_yaw;
  //   should_update_map_ = true;
  // }
}


// Timer-based map update
void MapMemoryNode::updateMap() {  
  if ((should_update_map_ && costmap_updated_)) {
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

  // float cos_yaw = std::cos(last_yaw);
  // float sin_yaw = std::sin(last_yaw);


  RCLCPP_INFO(this->get_logger(), "Entering integrateCostmap.");
  // bool has_non_zero_in_integrate = false;
  // for (int y_idx = 0; y_idx < latest_costmap_.info.height; ++y_idx) {
  //     for (int x_idx = 0; x_idx < latest_costmap_.info.width; ++x_idx) {
  //         int local_idx = y_idx * latest_costmap_.info.width + x_idx;
  //         if (latest_costmap_.data[local_idx] != 0 && latest_costmap_.data[local_idx] != -1) {
  //             has_non_zero_in_integrate = true;
  //             // If you want to see specific non-zero values, uncomment this:
  //             // RCLCPP_INFO(this->get_logger(), "Non-zero value found in latest_costmap_.data[%d][%d]: %d", x_idx, y_idx, latest_costmap_.data[local_idx]);
  //             break;
  //         }
  //     }
  //     if (has_non_zero_in_integrate) break;
  // }
  // RCLCPP_INFO(this->get_logger(), "latest_costmap_.data has non-zero values in integrateCostmap: %d", has_non_zero_in_integrate);

  for (int y_idx = 0; y_idx < latest_costmap_.info.height; ++y_idx) {
    for (int x_idx = 0; x_idx < latest_costmap_.info.width; ++x_idx) {
      // Cartesian coordinate in local frame
      // float x_local_cell = origin_x_local + x_idx*res; // maybe add 0.5f to get center of cell cartesian coordinate
      // float y_local_cell = origin_y_local + y_idx*res;

      // // Cartesian coordinate in global frame using robot position and orientation
      // float x_global_cell = curr_x + (cos_yaw * x_local_cell - sin_yaw * y_local_cell);
      // float y_global_cell = curr_y + (sin_yaw * x_local_cell + cos_yaw * y_local_cell);
      // float x_global_cell = (curr_x + origin_x_local) + (cos_yaw * x_local_cell - sin_yaw * y_local_cell);
      // float y_global_cell = (curr_y + origin_y_local) + (sin_yaw * x_local_cell + cos_yaw * y_local_cell) ;

      // 1. Position in robot frame
      double x_robot = origin_x_local + x_idx * res;
      double y_robot = origin_y_local + y_idx * res;

      // 2. Rotate to global frame
      double x_rot = x_robot * cos(curr_yaw) - y_robot * sin(curr_yaw);
      double y_rot = x_robot * sin(curr_yaw) + y_robot * cos(curr_yaw);

      // 3. Translate to global frame
      double x_global = curr_x + x_rot;
      double y_global = curr_y + y_rot;

      // // Check that cell is not out of bounds of global map
      // if (x_global_cell < 0 || y_global_cell < 0 || x_global_cell >= global_width_m || y_global_cell >= global_height_m) {
      //   continue; // Skip if out of bounds
      // }

      // Corresponding global_map_ indices for cell's global coordinate
      // 4. Convert to global map index
      int x_index = std::round((x_global - global_map_.info.origin.position.x) / res);
      int y_index = std::round((y_global - global_map_.info.origin.position.y) / res);

      // static_cast<int>(std::round((global_x - global_origin_x) / global_resolution));

      // if (x_idx == 0 && y_idx == 0) {
      //   RCLCPP_INFO(this->get_logger(), "Robot cartesian coordinate is %f, %f  yaw %f", curr_x, curr_y, curr_yaw);
      //   RCLCPP_INFO(this->get_logger(), "[0, 0] cartesian coordinate in robot frame is %f, %f", x_robot, y_robot);
      //   RCLCPP_INFO(this->get_logger(), "[0, 0] cartesian coordinate in global frame is %f, %f", x_global, y_global);
      // }
      
      int local_idx = y_idx * latest_costmap_.info.width + x_idx;
      int8_t value = latest_costmap_.data[local_idx];
      
      // if (value > 0) {
      //   RCLCPP_INFO(this->get_logger(), "Non-zero value at local index %d, %d", x_idx, y_idx);
      // }
      // if (local_idx == 48) {
      //   RCLCPP_INFO(this->get_logger(), "Value at index 48 %d", value);
      // }

      if (x_index < 0 || x_index >= global_map_.info.width || y_index < 0 || y_index >= global_map_.info.height) {
        if (value > 0) {
          RCLCPP_INFO(this->get_logger(), "Skipped non-zero value at local index %d, %d", x_idx, y_idx);
        }
        continue;
      }
      // int x_global_cell_idx = static_cast<int>((x_global_cell - global_map_.info.origin.position.x) / global_map_.info.resolution);
      // int y_global_cell_idx = static_cast<int>((y_global_cell - global_map_.info.origin.position.y) / global_map_.info.resolution);
      // // Check that indices are not out of bounds
      // if (x_global_cell_idx < 0 || y_global_cell_idx < 0 || x_global_cell_idx >= static_cast<int>(global_map_.info.width) || y_global_cell_idx >= static_cast<int>(global_map_.info.height)) {
      //   continue; // Skip if out of bounds
      // }

      // Convert to 1D global_map_ index
      // int global_idx = y_global_cell_idx * global_map_.info.width + x_global_cell_idx;
      // int8_t& global_val = global_map_.data[global_idx];
      int8_t global_val = global_map_.data[y_index * global_map_.info.width + x_index];

      
      
      // If a cell in the new costmap has a known value, overwrite its value into the global map
      if (value > global_val) {
        global_map_.data[y_index * global_map_.info.width + x_index] = value;
        // global_val = value;
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
