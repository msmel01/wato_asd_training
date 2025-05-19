#include <functional>
#include <unordered_map>
#include <queue>

using std::placeholders::_1;

#include "planner_node.hpp"


PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  state_ = State::WAITING_FOR_GOAL; // initialize state to waiting for goal point

  // Initialize subsciptions
  global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, _1));
  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::newGoalCallback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, _1));

  // Initialize publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}


void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_global_costmap_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      planPath(); // replan based on updated map
  }
}


void PlannerNode::newGoalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "Goal point at %f, %f", msg->point.x, msg->point.y);
  planPath();
}


void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}


void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
          // should we set goal_received_=false

      } else {
          RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
          planPath();

      }
  }
}


bool PlannerNode::goalReached() {
  double dx = goal_point_.point.x - robot_pose_.position.x;
  double dy = goal_point_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}


double PlannerNode::calculateHeuristic(int x, int y, int g_x, int g_y) {
  // Calculate manhattan distance from current node to goal node as heuristic
  return std::abs(x - g_x) + std::abs(y - g_y);
}

// Use current_global_costmap_ to see if a value > 0 is at index x, y -> obstacle
bool PlannerNode::isObstacle(int x, int y) {
  // Convert x,y into current_global_costmap_ array indices
  // int x_costmap_arr_idx = static_cast<int>((x - current_global_costmap_.info.origin.position.x) / current_global_costmap_.info.resolution);
  // int y_costmap_arr_idx = static_cast<int>((y- current_global_costmap_.info.origin.position.y) / current_global_costmap_.info.resolution);
  int costmap_arr_idx = y*current_global_costmap_.info.width + x;
  return current_global_costmap_.data[costmap_arr_idx] == 100; // obstacle score > 0 means there is obstacle
}


std::unordered_map<CellIndex, AStarNode, CellIndexHash> PlannerNode::AStarAlgorithm() {
  // TODO convert from double-type world coordinates into grid int-type grid coordinates
  // Current robot coordinates (in world frame)
  double x_robot = robot_pose_.position.x;
  double y_robot = robot_pose_.position.y;
  // Goal coordinates (in world frame)
  double x_goal = goal_point_.point.x;
  double y_goal = goal_point_.point.y;

  // Convert world coordinates to grid coordinates
  int x_robot_grid = static_cast<int>((x_robot - current_global_costmap_.info.origin.position.x) / current_global_costmap_.info.resolution);
  int y_robot_grid = static_cast<int>((y_robot - current_global_costmap_.info.origin.position.y) / current_global_costmap_.info.resolution);
  int x_goal_grid = static_cast<int>((x_goal - current_global_costmap_.info.origin.position.x) / current_global_costmap_.info.resolution);
  int y_goal_grid = static_cast<int>((y_goal - current_global_costmap_.info.origin.position.y) / current_global_costmap_.info.resolution);

  // Create AStarNode for robot starting cell
  CellIndex start_cell_index = CellIndex(x_robot_grid, y_robot_grid);
  AStarNode robot_start_node = AStarNode(start_cell_index, 0); // f_score of 0
  robot_start_node.g_score = 0;
  robot_start_node.parent_index = start_cell_index; // set to itself so we know when to stop when backtracking

  // Initialize explored as empty unordered_map
  std::unordered_map<CellIndex, AStarNode, CellIndexHash> explored;
  // Initialize to_explore_pq as an empty min priority queue
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> to_explore_pq;
  std::unordered_map<CellIndex, AStarNode, CellIndexHash> to_explore; // pq does not have search function so use to store/search for nodes in to_explore_pq

  // Add robot_start_node as first node to explore
  to_explore_pq.push(robot_start_node);
  // to_explore.insert(robot_start_node);
  to_explore[robot_start_node.index] = robot_start_node;


  while (!to_explore_pq.empty()) {
    // Get and remove next node we will explore with minimum f_score
    auto curr_node = to_explore_pq.top();
    to_explore_pq.pop(); // remove from pq
    to_explore.erase(curr_node.index); // remove from to_explore by using its CellIndex

    // Keep track of explored nodes
    // explored.insert(curr_node);
    explored[curr_node.index] = curr_node;

    // Look at all neighboring nodes
    for (int i_x = -1; i_x <= 1; ++i_x) {
      for (int i_y = -1; i_y <= 1; ++i_y) {
        if (i_x == 0 && i_y == 0) continue; // early stopping for self

        // Index of neighbor cell
        int n_x = curr_node.index.x + i_x;
        int n_y = curr_node.index.y + i_y;

        // Check for out of bounds index
        // float x_upper_boundary = (current_global_costmap_.info.width * current_global_costmap_.info.resolution) / 2;
        // float y_upper_boundary = (current_global_costmap_.info.height * current_global_costmap_.info.resolution) / 2;
        if (n_x < 0 || n_y < 0 || n_x >= static_cast<int>(current_global_costmap_.info.width) || n_y >= static_cast<int>(current_global_costmap_.info.height)) {
          continue;
        }

        // Create a new node for neighbor cells
        CellIndex neighbor_cell_index = CellIndex(n_x, n_y);
        AStarNode neighbor_node = AStarNode(neighbor_cell_index);

        // Check if neighbor node has already been fully explored
        auto it_neighbor_in_explored = explored.find(neighbor_cell_index);
        if (it_neighbor_in_explored != explored.end()) {
          continue;
        } 

        // Check to see if n_x, n_y contains an obstacle based on current_global_costmap_
        if (isObstacle(n_x, n_y)) {
          continue;
        }
        int costmap_arr_idx = n_y*current_global_costmap_.info.width + n_x;
        int cost = current_global_costmap_.data[costmap_arr_idx];

        // Populate h_score, g_score, and f_score for neighbor node
        neighbor_node.h_score = calculateHeuristic(n_x, n_y, x_goal_grid, y_goal_grid) + cost;
        neighbor_node.g_score = curr_node.g_score + 1; // assuming it costs 1 to step in any direction
        neighbor_node.f_score = neighbor_node.g_score + neighbor_node.h_score;

        // Check if this is the optimal path to this already visited node (is it in to_explore_pq?)
        auto it_neighbor_in_to_explore = to_explore.find(neighbor_cell_index);
        if (it_neighbor_in_to_explore != to_explore.end()) {
          if (neighbor_node.g_score > it_neighbor_in_to_explore->second.g_score) {
            // g_score of current path to neighbor_node is higher than a previously found path to neighbor_node 
            // Current path is not optimal
            continue;
          } else {
            // Update old neighbor_node from to_explore with new neighbor node values
            it_neighbor_in_to_explore->second.parent_index = curr_node.index;
            it_neighbor_in_to_explore->second.h_score = neighbor_node.h_score;
            it_neighbor_in_to_explore->second.g_score = neighbor_node.g_score;
            it_neighbor_in_to_explore->second.f_score = neighbor_node.f_score;
          }
        } else {
          // No previous nodes to this neighbor index existed so add this new neighbor_node to to_explore
          // Store CellIndex of parent node (aka curr_node) so that we can backtrack for the optimal path
          neighbor_node.parent_index = curr_node.index;
          to_explore_pq.push(neighbor_node);
          to_explore[neighbor_node.index] = neighbor_node;
        }
        
      }
    }
  }

  return explored;
}


void PlannerNode::planPath() {
  // RCLCPP_INFO(this->get_logger(), "Planning path");
  
  if (!goal_received_ || current_global_costmap_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
      return;
  }

  // Run A* to find path on current_global_costmap_
  std::unordered_map<CellIndex, AStarNode, CellIndexHash> explored = AStarAlgorithm();

  // Fill path.poses with the resulting waypoints: array of PoseStamped 
  // PoseStamped -> header and pose (position and quaternion)
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->get_clock()->now();

  // Backtrack to add the pose data of 
  std::vector<geometry_msgs::msg::PoseStamped> reverse_path;

  float resolution = current_global_costmap_.info.resolution;

  // Convert world coordinates to grid coordinates
  double x_robot = robot_pose_.position.x;
  double y_robot = robot_pose_.position.y;
  double x_goal = goal_point_.point.x;
  double y_goal = goal_point_.point.y;
  int x_robot_grid = static_cast<int>((x_robot - current_global_costmap_.info.origin.position.x) / resolution);
  int y_robot_grid = static_cast<int>((y_robot - current_global_costmap_.info.origin.position.y) / resolution);
  int x_goal_grid = static_cast<int>((x_goal - current_global_costmap_.info.origin.position.x) / resolution);
  int y_goal_grid = static_cast<int>((y_goal - current_global_costmap_.info.origin.position.y) / resolution);
  CellIndex start_cell_index(x_robot_grid, y_robot_grid);
  CellIndex goal_cell_index(x_goal_grid, y_goal_grid);

  if (explored.find(goal_cell_index) == explored.end()) {
    RCLCPP_WARN(this->get_logger(), "A* failed: goal not found in explored.");
    return;
  }

  CellIndex current_cell_index = goal_cell_index; // start at goal node

  while (current_cell_index != start_cell_index) {
    auto curr_node_it = explored.find(current_cell_index);

    if (curr_node_it == explored.end()) {
      RCLCPP_ERROR(this->get_logger(), "Node missing at (%d, %d).", current_cell_index.x, current_cell_index.y);
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = (current_cell_index.x * resolution + 0.05) + current_global_costmap_.info.origin.position.x; // convert to world coordinates
    pose.pose.position.y = (current_cell_index.y * resolution + 0.05) + current_global_costmap_.info.origin.position.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Path point at %f, %f", pose.pose.position.x, pose.pose.position.y);

    reverse_path.push_back(pose);
    current_cell_index = curr_node_it->second.parent_index; // backtrack to parent node
  }

  // Do we need to add start pose?
  std::reverse(reverse_path.begin(), reverse_path.end()); // reverse to get from start to goal pose

  path.poses = reverse_path; // no longer reverse!
  path_pub_->publish(path);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
