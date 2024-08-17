#include <iostream>
#include <stdio.h>
#include <math.h>
#include "stlastar.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>


class CostMap {

    const int width;
    const int height;
    const int resolution;
    const int origin;
    int** cost;


    CostMap()


};