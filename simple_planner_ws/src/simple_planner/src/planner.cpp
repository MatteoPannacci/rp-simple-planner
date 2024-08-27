#include "planner.h"


int max_nodes = 100000;


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

    std::queue<std::array<int,2>> queue;
    for(int i = 0; i < width; i++) {
        for(int j = 0; j < height; j++) {
            if(grid.data[i+j*width] == 100) {
                set(i,height-j-1, wall_cost);
                queue.push(std::array<int,2>{i,height-j-1});
            } 
        }
    }

    while(!queue.empty()) {
        std::array<int,2> coords = queue.front();
        queue.pop();
        propagate_wall_cost(coords[0], coords[1]);
    }

    reset(Eigen::Vector2f(origin.position.x, origin.position.y), resolution);

}


CostMap::~CostMap() {
    for(int i = 0; i < width; i++) {
        delete[] data[i];
    }
    delete[] data;
}


int CostMap::cost(int x, int y) {
    if(x < 0 || x >= width || y < 0 || y >= height) {
        return wall_cost + wall_cost_decay;
    }
    else {
        return data[x][y];
    }
}


void CostMap::set(int x, int y, int value) {
    if(x < 0 || x >= width || y < 0 || y >= height) {
        return;
    }
    else {
        data[x][y] = value;
    }
}


void CostMap::propagate_wall_cost(int x, int y) {
    if(x < 0 || x >= width || y < 0 || y >= height) {
        return;
    }
    else {
        int curr = cost(x,y);
        if(cost(x+1,y) < curr-wall_cost_decay) {
            set(x+1,y, curr-wall_cost_decay);
            propagate_wall_cost(x+1,y);
        }
        if(cost(x-1,y) < curr-wall_cost_decay) {
            set(x-1,y, curr-wall_cost_decay);
            propagate_wall_cost(x-1,y);
        }
        if(cost(x,y+1) < curr-wall_cost_decay) {
            set(x,y+1, curr-wall_cost_decay);
            propagate_wall_cost(x,y+1);
        }
        if(cost(x,y-1) < curr-wall_cost_decay) {
            set(x,y-1, curr-wall_cost_decay);
            propagate_wall_cost(x,y-1);
        } 
    }
}



SearchNode::SearchNode():
    x(0),
    y(0),
    map() {}


SearchNode::SearchNode(int x, int y, CostMap* map):
    x(x),
    y(y),
    map(map) {}


bool SearchNode::IsSameState(SearchNode& other) {
    if (x == other.x && y == other.y) {
        return true;
    }
    else {
        return false;
    }
}


size_t SearchNode::Hash() {
    size_t h1 = hash<float>{}(x);
    size_t h2 = hash<float>{}(y);
    return h1 ^ (h2 << 1);
}


float SearchNode::GoalDistanceEstimate(SearchNode& goal) {
    return abs(x - goal.x) + abs(y - goal.y);
}


bool SearchNode::IsGoal(SearchNode& goal) {
    if (x == goal.x && y == goal.y) {
        return true;
    }
    else {
        return false;
    }
}


bool SearchNode::GetSuccessors(AStarSearch<SearchNode>* astarsearch, SearchNode* parent) {

    int parent_x = -1;
    int parent_y = -1;
    if(parent) {
        int parent_x = parent->x;
        int parent_y = parent->y;
    }

    if ((map->cost(x-1,y) < map->wall_cost) && !((parent_x == x-1) && (parent_y == y))) {
        SearchNode child = SearchNode(x-1,y,map);
        astarsearch->AddSuccessor(child);
    }

    if ((map->cost(x+1,y) < map->wall_cost) && !((parent_x == x+1) && (parent_y == y))) {
        SearchNode child = SearchNode(x+1,y,map);
        astarsearch->AddSuccessor(child);
    }

    if ((map->cost(x,y-1) < map->wall_cost) && !((parent_x == x) && (parent_y == y-1))) {
        SearchNode child = SearchNode(x,y-1,map);
        astarsearch->AddSuccessor(child);
    }

    if ((map->cost(x,y+1) < map->wall_cost) && !((parent_x == x) && (parent_y == y+1))) {
        SearchNode child = SearchNode(x,y+1,map);
        astarsearch->AddSuccessor(child);
    }
    
    return true;

}


float SearchNode::GetCost(SearchNode& successor) {
    return (float) map->cost(x,y) + map->step_cost;
}



void Planner::set_map(nav_msgs::OccupancyGrid grid, int wall_cost, int wall_cost_decay, int step_cost) {
    map = new CostMap(grid, wall_cost, wall_cost_decay, step_cost);
}


void Planner::set_start(geometry_msgs::Point start_point) {
    Eigen::Vector2f start_vector(start_point.x, start_point.y);
    Eigen::Vector2f start_grid_vector = map->world2grid(start_vector);
    int x = (int) floor(start_grid_vector[0]);
    int y = (int) floor(start_grid_vector[1]);
    start = SearchNode(x,y,map);
}


void Planner::set_goal(geometry_msgs::Point goal_point) {
    Eigen::Vector2f goal_vector(goal_point.x, goal_point.y);
    Eigen::Vector2f goal_grid_vector = map->world2grid(goal_vector);
    int x = (int) floor(goal_grid_vector[0]);
    int y = (int) floor(goal_grid_vector[1]);
    goal = SearchNode(x,y,map);
}


void Planner::find_path() {

    path = std::vector<array<int,2>>();
    AStarSearch<SearchNode> astarsearch(max_nodes);
    astarsearch.SetStartAndGoalStates(start, goal);
    unsigned int SearchState;
    int counter = 0;
    
    do {
        SearchState = astarsearch.SearchStep();
        counter++;
        if(counter% int(max_nodes/100) == 0) {
            std::cout << "iteration n. " << counter << std::endl;
        }
    } while(SearchState == AStarSearch<SearchNode>::SEARCH_STATE_SEARCHING);

    if(SearchState == AStarSearch<SearchNode>::SEARCH_STATE_SUCCEEDED) {
        SearchNode* node = astarsearch.GetSolutionStart();
        while(node != NULL) {
            std::array<int,2> coord{node->x, node->y};
            path.push_back(coord);
            node = astarsearch.GetSolutionNext();
        }
        std::cout << "last iteration n." << counter << std::endl;
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