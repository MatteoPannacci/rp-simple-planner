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