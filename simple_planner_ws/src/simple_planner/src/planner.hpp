#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "stlastar.h"


class CostMap {

public:

    int width;
    int height;
    float resolution;
    geometry_msgs::Pose origin;
    int wall_cost;
    int** data;


    CostMap();
    CostMap(nav_msgs::OccupancyGrid grid, int wall_cost);
    ~CostMap();

    int cost(int r, int c);
    void set(int r, int c, int value);
    void propagate_wall_cost(int r, int c);

};


class SearchNode {

public:

    int r;
    int c;
    CostMap map;

    SearchNode();
    SearchNode(int r, int c, CostMap& map);
    float GoalDistanceEstimate(SearchNode& goal);
    bool IsGoal(SearchNode& goal);
    bool GetSuccessors(AStarSearch<SearchNode>* astarsearch, SearchNode* parent);
    float GetCost(SearchNode& node);
    bool IsSameState(SearchNode& other);
    size_t Hash();
    void PrintNodeInfo();

};


class Planner {

public:

    CostMap map;
    SearchNode start;
    SearchNode goal;
    nav_msgs::Path path;


    void set_map(nav_msgs::OccupancyGrid grid, int wall_cost);
    void set_start(geometry_msgs::Point start_point);
    void set_goal(geometry_msgs::Point goal_point);
    void find_path();
    nav_msgs::Path get_path();

};