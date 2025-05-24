#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "control_node.hpp"

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller") {
  // Initialize parameters
  lookahead_distance_ = 1.0;  // Lookahead distance
  goal_tolerance_ = 0.2;     // Distance to consider the goal reached
  linear_speed_ = 1;       // Constant forward speed

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void PurePursuitController::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
      return;
  }

  // Check if close to goal?
  const auto &goal = current_path_->poses.back().pose.position;
  const auto &robot_pos = robot_odom_->pose.pose.position;
  if (computeDistance(robot_pos, goal) <= goal_tolerance_) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    return;
  }


  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    return ;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}


std::optional<geometry_msgs::msg::PoseStamped> PurePursuitController::findLookaheadPoint() {
  for (const auto& pose: current_path_->poses) {
    double distance = computeDistance(robot_odom_->pose.pose.position, pose.pose.position);
    if (distance >= lookahead_distance_) return pose;
  }
  // Otherwise return null (no valid lookahead point found)        
  return std::nullopt;
}

geometry_msgs::msg::Twist PurePursuitController::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  // RCLCPP_INFO(this->get_logger(), "Current robot point at %f, %f", robot_odom_->pose.pose.position.x, robot_odom_->pose.pose.position.y);
  // RCLCPP_INFO(this->get_logger(), "Lookahead point at %f, %f", target.pose.position.x, target.pose.position.y);

  const auto &goal = current_path_->poses.back().pose.position;
  const auto &robot_pos = robot_odom_->pose.pose.position;
  if (computeDistance(robot_pos, goal) <= goal_tolerance_) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }
  
  const auto& robot_position = robot_odom_->pose.pose.position;
  const auto& robot_orientation = robot_odom_->pose.pose.orientation;
  double current_yaw = extractYaw(robot_orientation);

  const auto& target_position = target.pose.position;
  const auto& target_orientation = target.pose.orientation;

  const tf2::Quaternion target_q(
    target.pose.orientation.x,
    target.pose.orientation.y,
    target.pose.orientation.z,
    target.pose.orientation.w
  );

  const geometry_msgs::msg::Quaternion target_quat = tf2::toMsg(target_q);
  //tf2::Quaternion relative_q = target_q * inverse_robot_q;

  double angle_to_target = std::atan2(target_position.y - robot_position.y, target_position.x - robot_position.x);

  //const geometry_msgs::msg::Quaternion relative_q_quat = tf2::toMsg(relative_q);

  double steering_angle = angle_to_target - current_yaw;

  //cmd_vel.angular.x = std::cos(steering_angle) * linear_speed_;
  //cmd_vel.angular.y = std::sin(steering_angle) * linear_speed_;

  while (steering_angle > M_PI) steering_angle -= 2 * M_PI;
  while (steering_angle < -M_PI) steering_angle += 2 * M_PI;

  // Robot's linear speed is kept constant
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = 1.4 * steering_angle;

  return cmd_vel;
}



double PurePursuitController::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(a.y - b.y, a.x - b.x);
}

double PurePursuitController::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  return std::atan2(2*(quat.w*quat.z + quat.x*quat.y), 1 - 2*(quat.y*quat.y + quat.z*quat.z));
}


// Main function
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}