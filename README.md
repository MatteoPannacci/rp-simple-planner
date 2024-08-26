# rp-simple-planner

Implementation of a simple 2D planner node for ROS in C++. This project is part of the exam of Robot Programming for the Master Degree in Artificial Intelligence and Robotics at Sapienza, academic year 2023/2024.


## How to use

1)  go to the simple_planner_ws directory

2)  add the package to the catkin setup directories

3)  run the roscore node

4)  run the stage_ros node

5)  run the map_server node

6)  run the goal_publisher_node node

7)  run the simple_planner_node node

        rosrun simple_planner simple_planner_node \<wall_cost\> \<wall_cost_decay\> \<step_cost\>
        

## Requirements

...





The A* algorithm implementation is from [text](https://github.com/justinhj/astar-algorithm-cpp)