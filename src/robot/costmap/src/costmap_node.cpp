#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CostmapNode::publishCostmap, this));

  width_ = 60 / resolution_;
  height_ = 60 / resolution_;
  origin_x_ = -30;
  origin_y_ = -30;
  
  grid_msg_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid_msg_->info.resolution = resolution_;
  grid_msg_->info.width = width_;
  grid_msg_->info.height = height_;
  grid_msg_->info.origin.position.x = origin_x_;
  grid_msg_->info.origin.position.y = origin_y_;
  grid_msg_->info.origin.position.z = 0.0;
  grid_msg_->info.origin.orientation.w = 1.0;
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap();
    grid_msg_->header.stamp = now();
    grid_msg_->header.frame_id = scan->header.frame_id;

 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}

void CostmapNode::initializeCostmap() {
    // RCLCPP_INFO(logger_, "Initializing costmap with width: %d and height: %d", width_costmap_arr_, height_costmap_arr_);
    costmap = std::vector<std::vector<int>>(height_, std::vector<int>(width_, 0));
    grid_msg_->data.clear();
    grid_msg_->data.resize(
        grid_msg_->info.width * grid_msg_->info.height, 0
    );
}

void CostmapNode::convertToGrid(float range, float angle, int &x_grid, int &y_grid){
    // map's resolution
    x_grid = (range * cos(angle) - origin_x_) / resolution_;
    y_grid = (range * sin(angle) - origin_y_) / resolution_;
}


void CostmapNode::markObstacle(int x_grid, int y_grid) {
    // this->costmap[y_grid][x_grid] = this->obstacle_cost_;

    if (x_grid > 0 && x_grid < width_ && y_grid > 0 && y_grid < height_) {
        costmap[static_cast<int>(y_grid)][static_cast<int>(x_grid)] = obstacle_cost_;
    }
}

void CostmapNode::inflateObstacles() {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (costmap[y][x] == obstacle_cost_) {
                // This is an obstacle cell

                for (int dy = -inflation_radius_cells_; dy <= inflation_radius_cells_; ++dy) {
                    for (int dx = -inflation_radius_cells_; dx <= inflation_radius_cells_; ++dx) {
                        int x_neighbor = x + dx; // index of neighbor in costmap array
                        int y_neighbor = y + dy;
                        
                        if (x_neighbor < 0 || x_neighbor >= width_ || y_neighbor < 0 || y_neighbor >= height_) {
                            // Skip if index out of bounds of array
                            continue;
                        }
                        
                        // (x - x_neighbor)^2 + (y - y_neighbor)^2 = (x - (x + dx))^2 + (y - (y + dy))^2 
                        float euclidean = std::sqrt(dx*dx+ dy*dy) * resolution_;

                        if (euclidean > inflation_radius_ || euclidean == 0.0f) {
                            continue; // Make sure within radius
                        }

                        int inflated_cost = static_cast<int>(max_inflation_cost_ * (1.0f - euclidean / inflation_radius_));

                        // // NEW
                        // size_t x_i = static_cast<size_t>(x_neighbor);
                        // size_t y_i = static_cast<size_t>(y_neighbor);

                        if (inflated_cost > costmap[y_neighbor][x_neighbor]) {
                            costmap[y_neighbor][x_neighbor] = inflated_cost;
                        }
                    }
                }

                // RCLCPP_INFO(logger_, "Obstacle at grid (%d, %d)", x, y);
            }
        }
    }
}
 
void CostmapNode::publishCostmap() {
    for (size_t y = 0; y < grid_msg_->info.height; ++y) {
        for (size_t x = 0; x < grid_msg_->info.width; ++x) {
          int value = costmap[y][x];
          int8_t mapped_value = (value >= 100) ? 100 : value; // clamp to max 100 and data type required for occupancy grid
          grid_msg_->data[y * grid_msg_->info.width + x] = mapped_value;
        }
      }
    
    costmap_pub_->publish(*(grid_msg_));
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}