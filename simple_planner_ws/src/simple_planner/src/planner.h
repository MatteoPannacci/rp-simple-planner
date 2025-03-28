#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "stlastar.h"
#include "grid_map/grid_map.h"


class CostMap: public GridMapping {

public:

    int width;
    int height;
    float resolution;
    geometry_msgs::Pose origin;
    int wall_cost;
    int wall_cost_decay;
    int step_cost;
    int** data;


    CostMap();
    CostMap(nav_msgs::OccupancyGrid grid, int wall_cost, int wall_cost_decay, int step_cost);
    ~CostMap();

    int cost(int x, int y);
    void set(int x, int y, int value);
    void propagate_wall_cost(int x, int y);

};


class SearchNode {

public:

    int x;
    int y;
    CostMap* map;

    SearchNode();
    SearchNode(int x, int y, CostMap* map);
    float GoalDistanceEstimate(SearchNode& goal);
    bool IsGoal(SearchNode& goal);
    bool GetSuccessors(AStarSearch<SearchNode>* astarsearch, SearchNode* parent);
    float GetCost(SearchNode& node);
    bool IsSameState(SearchNode& other);
    size_t Hash();

};


class Planner {

public:

    CostMap* map;
    SearchNode start;
    SearchNode goal;
    std::vector<std::array<int,2>> path;


    void set_map(nav_msgs::OccupancyGrid grid, int wall_cost, int wall_cost_decay, int step_cost);
    void set_start(geometry_msgs::Point start_point);
    void set_goal(geometry_msgs::Point goal_point);
    void find_path();
    nav_msgs::Path get_path();

};