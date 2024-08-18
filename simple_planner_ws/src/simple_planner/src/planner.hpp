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

};


class SearchNode {

    int r;
    int c;

public:

    float GoalDistanceEstimate(SearchNode& goal);
    bool isGoal(SearchNode& node);
    bool GetSuccessor(AStarSearch<SearchNode>* astarsearch, SearchNode *parent);
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
    void set_start(const geometry_msgs::PoseStamped start);
    void set_goal(const geometry_msgs::PoseStamped goal);
    void search();

};