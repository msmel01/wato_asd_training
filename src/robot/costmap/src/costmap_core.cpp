#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}


void CostmapCore::initializeGrid() {
    // RCLCPP_INFO(logger_, "Initializing costmap with width: %d and height: %d", width_costmap_arr_, height_costmap_arr_);
    // costmap = std::vector<std::vector<int>>(height_costmap_arr_, std::vector<int>(width_costmap_arr_, 0));
    // Store as 1D array
    costmap = std::vector<int>(height_costmap_arr_ * width_costmap_arr_, 0);
}


float CostmapCore::getResolution() {
    return this->resolution_;
}


int CostmapCore::getWidth() {
    return this->width_costmap_arr_;
}


int CostmapCore::getHeight() {
    return this->height_costmap_arr_;
}


float CostmapCore::getWidthWorld() {
    return this->width_world_;
}


float CostmapCore::getHeightWorld() {
    return this->height_world_;
}


int CostmapCore::getCost(size_t x, size_t y) {
    // return this->costmap[y][x];
    // REFACTORED FOR 1D costmap
    return this->costmap[y*width_costmap_arr_ + x];
}

void CostmapCore::convertToGrid(float range, float angle, int& x_grid, int& y_grid) {
    float x = range * std::cos(angle);
    float y = range * std::sin(angle);
    float origin_x = - width_world_ / 2.0f;
    float origin_y = - height_world_ / 2.0f;

    x_grid = static_cast<int>(std::round((x - origin_x) / resolution_));
    y_grid = static_cast<int>(std::round((y - origin_y) / resolution_));
}


void CostmapCore::markObstacle(int x_grid, int y_grid) {
    // this->costmap[y_grid][x_grid] = this->obstacle_cost_;

    // if (x_grid >= 0 && x_grid < width_costmap_arr_ && y_grid >= 0 && y_grid < height_costmap_arr_) {
    //     costmap[static_cast<int>(y_grid)][static_cast<int>(x_grid)] = obstacle_cost_;
    // } else {
    //     // RCLCPP_WARN(logger_, "markObstacle out of bounds: (%d, %d)", x_grid, y_grid);
    // }
    // REFACTORED FOR 1D costmap
    if (x_grid >= 0 && x_grid < width_costmap_arr_ && y_grid >= 0 && y_grid < height_costmap_arr_) {
        costmap[y_grid * width_costmap_arr_ + x_grid] = obstacle_cost_;
    } else {
        // RCLCPP_WARN(logger_, "markObstacle out of bounds: (%d, %d)", x_grid, y_grid);
    }

}


void CostmapCore::inflateObstacles() {
    for (int y = 0; y < height_costmap_arr_; ++y) {
        for (int x = 0; x < width_costmap_arr_; ++x) {
            // if (costmap[y][x] == obstacle_cost_) {

            // REFACTORED FOR 1D costmap
            if (costmap[y * width_costmap_arr_ + x] == obstacle_cost_) {
                // This is an obstacle cell

                for (int dy = -inflation_radius_cells_; dy <= inflation_radius_cells_; ++dy) {
                    for (int dx = -inflation_radius_cells_; dx <= inflation_radius_cells_; ++dx) {
                        int x_neighbor = x + dx; // index of neighbor in costmap array
                        int y_neighbor = y + dy;
                        
                        if (x_neighbor < 0 || x_neighbor >= width_costmap_arr_ || y_neighbor < 0 || y_neighbor >= height_costmap_arr_) {
                            // Skip if index out of bounds of array
                            continue;
                        }
                        
                        // (x - x_neighbor)^2 + (y - y_neighbor)^2 = (x - (x + dx))^2 + (y - (y + dy))^2 
                        float euclidean = std::sqrt(dx*dx+ dy*dy) * resolution_;

                        if (euclidean > inflation_radius_ || euclidean == 0.0f) {
                            continue; // Make sure within radius
                        }

                        int inflated_cost = static_cast<int>(max_inflation_cost_ * (1.0f - euclidean / inflation_radius_));

                        // if (inflated_cost > costmap[y_neighbor][x_neighbor]) {
                        //     costmap[y_neighbor][x_neighbor] = inflated_cost;
                        // }
                        // 
                        // REFACTORED FOR 1D costmap
                        if (inflated_cost > costmap[y_neighbor * width_costmap_arr_ + x_neighbor]) {
                            costmap[y_neighbor * width_costmap_arr_ + x_neighbor] = inflated_cost;
                        }
                    }
                }

                // RCLCPP_INFO(logger_, "Obstacle at grid (%d, %d)", x, y);
            }
        }
    }
}

}