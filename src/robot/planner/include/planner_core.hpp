#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

/* ------------------- A* graph supporting structs ------------------- */ 
// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  CellIndex parent_index;
  double h_score;  // heuristics value to get to end node
  double g_score;  // g value to get from the start node
  double f_score;  // f = g + h

  AStarNode() : index(0, 0), parent_index(0, 0) {}

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}

  // Set f_score later
  AStarNode(CellIndex idx) : index(idx) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};


namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

  private:
    rclcpp::Logger logger_;

    
};

}  

#endif  
