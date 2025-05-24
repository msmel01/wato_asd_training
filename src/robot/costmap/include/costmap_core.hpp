#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeGrid();

    void convertToGrid(float range, float angle, int& x_grid, int& y_grid); // Calculates grid coordinates

    void markObstacle(int x_grid, int y_grid); // Sets value at grid coordinates to high obstacle_cost_ value

    void inflateObstacles();

    // Getter functions
    float getResolution();

    int getWidth();

    int getHeight();

    float getWidthWorld();

    float getHeightWorld();

    int getCost(size_t x, size_t y);

  private:
    rclcpp::Logger logger_;

    // Based on robot_env.sdf environment bounds: [-15.25, 15.25]
    static constexpr float width_world_ = 30.5f;
    static constexpr float height_world_ = 30.5f;

    static constexpr float resolution_ = 0.1; // 0.1 m per cell

    // Define dimensions of costmap and initialize array
    static constexpr int width_costmap_arr_ = 305; // 30.5 / 0.1
    static constexpr int height_costmap_arr_ = 305;
    // std::vector<std::vector<int>> costmap;
    std::vector<int> costmap;

    // Value set to cells with an obstacle
    static constexpr int obstacle_cost_ = 100;

    // Inflation radius and costs
    static constexpr float inflation_radius_ = 1.5f;
    static constexpr int inflation_radius_cells_ = static_cast<int>(inflation_radius_ / resolution_);
    static constexpr int max_inflation_cost_ = 100; // make the cost must higher to give higher weight to obstacles
};

}  

#endif  