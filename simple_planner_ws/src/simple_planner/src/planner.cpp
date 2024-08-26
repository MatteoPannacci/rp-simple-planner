#include "planner.h"


CostMap::CostMap():
    width(0),
    height(0),
    resolution(0),
    origin(),
    wall_cost(0)
{
    data = NULL;
} 


CostMap::CostMap(const nav_msgs::OccupancyGrid grid, int wall_cost, int wall_cost_decay, int step_cost):
    width(grid.info.width),
    height(grid.info.height),
    resolution(grid.info.resolution),
    origin(grid.info.origin),
    wall_cost(wall_cost),
    wall_cost_decay(wall_cost_decay),
    step_cost(step_cost)
{
    data = new int*[width]();
    for(int i = 0; i < width; i++) {
        data[i] = new int[height]();
    }

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if(grid.data[i*width+j] == 100) {
                set(j,height-i-1, wall_cost);
            } 
        }
    }

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if(grid.data[i*width+j] == 100) {
                propagate_wall_cost(j,height-i-1);
            } 
        }
    }


    reset(Eigen::Vector2f(origin.position.x, origin.position.y), resolution);

}


CostMap::~CostMap() {
    for(int i = 0; i < width; i++) {
        delete[] data[i];
    }
    delete[] data;
}


int CostMap::cost(int r, int c) {
    if(r < 0 || r >= width || c < 0 || c >= height) {
        return wall_cost+wall_cost_decay;
    }
    else {
        return data[r][c];
    }
}


void CostMap::set(int r, int c, int value) {
    if(r < 0 || r >= width || c < 0 || c >= height) {
        return;
    }
    else {
        data[r][c] = value;
    }
}


void CostMap::propagate_wall_cost(int r, int c) {
    if(r < 0 || r >= width || c < 0 || c >= height) {
        return;
    }
    else {
        int curr = cost(r,c);
        if(cost(r+1,c) < curr-wall_cost_decay) {
            set(r+1,c, curr-wall_cost_decay);
            propagate_wall_cost(r+1,c);
        }
        if(cost(r-1,c) < curr-wall_cost_decay) {
            set(r-1,c, curr-wall_cost_decay);
            propagate_wall_cost(r-1,c);
        }
        if(cost(r,c+1) < curr-wall_cost_decay) {
            set(r,c+1, curr-wall_cost_decay);
            propagate_wall_cost(r,c+1);
        }
        if(cost(r,c-1) < curr-wall_cost_decay) {
            set(r,c-1, curr-wall_cost_decay);
            propagate_wall_cost(r,c-1);
        } 
    }
}



SearchNode::SearchNode():
    r(0),
    c(0),
    map() {}


SearchNode::SearchNode(int r, int c, CostMap* map):
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


bool SearchNode::GetSuccessors(AStarSearch<SearchNode>* astarsearch, SearchNode* parent) {

    int parent_r = -1;
    int parent_c = -1;
    if(parent) {
        int parent_r = parent->r;
        int parent_c = parent->c;
    }

    if ((map->cost(r-1,c) < map->wall_cost) && !((parent_r == r-1) && (parent_c == c))) {
        SearchNode child = SearchNode(r-1,c,map);
        astarsearch->AddSuccessor(child);
    }

    if ((map->cost(r+1,c) < map->wall_cost) && !((parent_r == r+1) && (parent_c == c))) {
        SearchNode child = SearchNode(r+1,c,map);
        astarsearch->AddSuccessor(child);
    }

    if ((map->cost(r,c-1) < map->wall_cost) && !((parent_r == r) && (parent_c == c-1))) {
        SearchNode child = SearchNode(r,c-1,map);
        astarsearch->AddSuccessor(child);
    }

    if ((map->cost(r,c+1) < map->wall_cost) && !((parent_r == r) && (parent_c == c+1))) {
        SearchNode child = SearchNode(r,c+1,map);
        astarsearch->AddSuccessor(child);
    }
    
    return true;

}


float SearchNode::GetCost(SearchNode& successor) {
    return (float) map->cost(r,c) + map->step_cost;
}



void Planner::set_map(nav_msgs::OccupancyGrid grid, int wall_cost, int wall_cost_decay, int step_cost) {
    map = new CostMap(grid, wall_cost, wall_cost_decay, step_cost);
}


void Planner::set_start(geometry_msgs::Point start_point) {
    Eigen::Vector2f start_vector(start_point.x, start_point.y);
    Eigen::Vector2f start_grid_vector = map->world2grid(start_vector);
    int r = (int) floor(start_grid_vector[0]);
    int c = (int) floor(start_grid_vector[1]);
    start = SearchNode(r,c,map);
}


void Planner::set_goal(geometry_msgs::Point goal_point) {
    Eigen::Vector2f goal_vector(goal_point.x, goal_point.y);
    Eigen::Vector2f goal_grid_vector = map->world2grid(goal_vector);
    int r = (int) floor(goal_grid_vector[0]);
    int c = (int) floor(goal_grid_vector[1]);
    goal = SearchNode(r,c,map);
}


void Planner::find_path() {

    path = std::vector<array<int,2>>();
    AStarSearch<SearchNode> astarsearch(100000);
    astarsearch.SetStartAndGoalStates(start, goal);
    unsigned int SearchState;
    int counter = 0;
    
    do {
        SearchState = astarsearch.SearchStep();
        counter++;
        if(counter%100 == 0) {
            std::cout << "iteration n. " << counter << std::endl;
        }
    } while(SearchState == AStarSearch<SearchNode>::SEARCH_STATE_SEARCHING);

    if(SearchState == AStarSearch<SearchNode>::SEARCH_STATE_SUCCEEDED) {
        SearchNode* node = astarsearch.GetSolutionStart();
        while(node != NULL) {
            std::array<int,2> coord{node->r, node->c};
            path.push_back(coord);
            std::cout << "coord: " << node->r << " " << node->c << std::endl;
            std::cout << "cost: " << map->cost(node->r, node->c) <<  std::endl << std::endl;
            node = astarsearch.GetSolutionNext();
        }
    }

}


nav_msgs::Path Planner::get_path() {

    nav_msgs::Path path_message;
    int lenght = path.size();
    std::vector<geometry_msgs::PoseStamped> poses;

    for(int i = 0; i < lenght; i++) {
        geometry_msgs::PoseStamped pose_stamped;
        Eigen::Vector2f grid_position_vector(path[i][0], path[i][1]);
        Eigen::Vector2f world_position_vector;
        world_position_vector = map->grid2world(grid_position_vector);
        pose_stamped.pose.position.x = world_position_vector[0];
        pose_stamped.pose.position.y = world_position_vector[1];
        poses.push_back(pose_stamped);
    }
    path_message.poses = poses;

    return path_message;

}