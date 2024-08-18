#include "planner.hpp"


CostMap::CostMap(const nav_msgs::OccupancyGrid grid, const int wall_cost):
    width(grid.info.width),
    height(grid.info.height),
    resolution(grid.info.resolution),
    origin(grid.info.origin),
    wall_cost(wall_cost) 
{
    data = new int*[height]();
    for(int i = 0; i < height; i++) {
        data[i] = new int[width]();
    }

    // map_server OccupancyGrid values can be 0, 100 or -1
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if(grid.data[i*width+j] == 100) {
                set(i,j, wall_cost);
                propagate_wall_cost(i,j);
            } 
        }
    }

}


CostMap::~CostMap() {
    for(int i = 0; i < height; i++) {
        delete[] data[i];
    }
    delete[] data;
}


int CostMap::cost(int r, int c) {
    if(r < 0 || r >= height || c < 0 || c >= width) {
        return wall_cost+1;
    }
    else {
        return data[r][c];
    }
}


void CostMap::set(int r, int c, int value) {
    if(r < 0 || r >= height || c < 0 || c >= width) {
        return;
    }
    else {
        data[r][c] = value;
    }
}


void CostMap::propagate_wall_cost(int r, int c) {
    if(r < 0 || r >= height || c < 0 || c >= width) {
        return;
    }
    else {
        int curr = data[r][c];
        if(cost(r+1,c) < curr) {
            set(r+1,c, curr-1);
            propagate_wall_cost(r+1,c);
        }
        if(cost(r-1,c) < curr) {
            set(r-1,c, curr-1);
            propagate_wall_cost(r-1,c);
        }
        if(cost(r,c+1) < curr) {
            set(r,c+1, curr-1);
            propagate_wall_cost(r,c+1);
        }
        if(cost(r,c-1) < curr) {
            set(r,c-1, curr-1);
            propagate_wall_cost(r,c-1);
        } 
    }
}



SearchNode::SearchNode(int r, int c, CostMap& map):
    r(r),
    c(c),
    map(map) {}


bool SearchNode::IsSameState(SearchNode& other) {
    if (r == other.r && c == other.c) {
        return true;
    }
    else {
        return false;
    }
}


size_t SearchNode::Hash() {
    size_t h1 = hash<float>{}(r);
    size_t h2 = hash<float>{}(c);
    return h1 ^ (h2 << 1);
}


float SearchNode::GoalDistanceEstimate(SearchNode& goal) {
    return abs(r - goal.r) + abs(c - goal.c);
}


bool SearchNode::IsGoal(SearchNode& goal) {
    if (r == goal.r && c == goal.c) {
        return true;
    }
    else {
        return false;
    }
}


bool SearchNode::GetSuccessors(AStarSearch<SearchNode>* astarsearch, SearchNode *parent) {

    int parent_r = parent->r;
    int parent_c = parent->c;
    CostMap& map = parent->map;

    if (map.cost(r-1,c) < map.wall_cost) {
        SearchNode child = SearchNode(r,c,map);
        astarsearch->AddSuccessor(child);
    }

    if (map.cost(r+1,c) < map.wall_cost) {
        SearchNode child = SearchNode(r,c,map);
        astarsearch->AddSuccessor(child);
    }

    if (map.cost(r,c-1) < map.wall_cost) {
        SearchNode child = SearchNode(r,c,map);
        astarsearch->AddSuccessor(child);
    }

    if (map.cost(r,c+1) < map.wall_cost) {
        SearchNode child = SearchNode(r,c,map);
        astarsearch->AddSuccessor(child);
    }
    
    return true;

}


float SearchNode::GetCost(SearchNode& successor) {
    return (float) map.cost(r,c);
}



void Planner::set_map(const nav_msgs::OccupancyGrid grid, const int wall_cost) {
    CostMap map(grid, wall_cost);
}


void Planner::set_start(const geometry_msgs::PoseStamped start_pose) {
    double x = start_pose.pose.position.x;
    double y = start_pose.pose.position.y;    
    int r = int(floor((x - map.origin.position.x) / map.resolution));
    int c = int(floor((y - map.origin.position.y) / map.resolution));
    SearchNode start(r,c,map);

}


void Planner::set_goal(const geometry_msgs::PoseStamped goal_pose) {
    double x = goal_pose.pose.position.x;
    double y = goal_pose.pose.position.y;    
    int r = int(floor((x - map.origin.position.x) / map.resolution));
    int c = int(floor((y - map.origin.position.y) / map.resolution));
    SearchNode goal(r,c,map);
}


void Planner::search() {
    return;
}