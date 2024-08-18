#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "stlastar.h"


class CostMap {

    const int width;
    const int height;
    const float resolution;
    const geometry_msgs::Pose origin;
    const int wall_cost;
    int** data;

public:

    CostMap(const nav_msgs::OccupancyGrid grid, const int wall_cost);
    ~CostMap();

    int cost(int r, int c);
    void set(int r, int c, int value);
    void propagate_wall_cost(int r, int c);
    int get_wall_cost();

};


class SearchNode {

    int r;
    int c;
    CostMap& map;

public:

    SearchNode(int r, int c, CostMap& map);
    float GoalDistanceEstimate(SearchNode& goal);
    bool IsGoal(SearchNode& goal);
    bool GetSuccessors(AStarSearch<SearchNode>* astarsearch, SearchNode *parent);
    float GetCost(SearchNode& node);
    bool IsSameState(SearchNode& other);
    size_t Hash();
    void PrintNodeInfo(); // change with return coordinates

};


class Planner {

    CostMap map;
    SearchNode start;
    SearchNode goal;

public:

    void set_map(const nav_msgs::OccupancyGrid grid, const int wall_cost);
    void set_start(const geometry_msgs::PoseStamped start_pose);
    void set_goal(const geometry_msgs::PoseStamped goal_pose);
    void search();

};