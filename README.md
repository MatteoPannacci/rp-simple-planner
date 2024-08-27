# rp-simple-planner

### Summary

Implementation of a simple 2D planner node for ROS in C++. This project is part of the exam of Robot Programming for the Master Degree in Artificial Intelligence and Robotics at Sapienza, academic year 2023/2024.

The repository provides:

goal_publisher_node: a node that publishes 'geometry_msgs/PoseStamped' messages with the goal position.

planner: builds a grid structure from the map message, locate the start and goal position within it and applies A* to find the best path between the two points.

simple_planner_node: computes the start position from the 'tf2_msgs/TFMessage' provided by the simulator, reads the map structure from the 'nav:msgs/OccupancyGrid' message provided by the map_server and the goal position from the 'geometry_msgs/PoseStamped' message provided by the goal publisher node. Then it uses these informations to build the planner and from it obtains the 'nav_msgs/Path' message of the identified path which is visualized with OpenCV.

/grid_map: set of classes used for the visualization of the path and the transformation between the grid structure and the world coordinates.


### Compilation

1)  setup the ROS environment

        source /opt/ros/noetic/setup.bash

2)  go to the simple_planner_ws directory

3)  compile using catkin

        catkin build


### How to use

1)  setup the ROS environment

        source /opt/ros/noetic/setup.bash
    
3)  go to the simple_planner_ws directory

4)  add the package to the catkin setup directories

        source ./devel/setup.bash

5)  run the roscore node

        roscore

6)  run the stage_ros simulator node with the provided .world file

        rosrun stage_ros stageros ./data/cappero.world

7)  run the map_server map publisher node with the provided .yaml file

        rosrun map_server map_server ./data/cappero.yaml

8)  run the goal_publisher_node node with the coordinates of the destination

        rosrun simple_planner goal_publisher_node <goal_x> <goal_y> <frequency>

9)  run the simple_planner_node node with the parameters for the planning

        rosrun simple_planner simple_planner_node <wall_cost> <wall_cost_decay> <step_cost>
        

### Requirements

To compile and run the code the following are required:

*   [ROS noetic](https://wiki.ros.org/noetic)
*   [OpenCV](https://opencv.org)
*   [The ROS Navigation Stack](https://github.com/ros-planning/navigation) (for the map_server node)
*   [justinhj's A* implementation](https://github.com/justinhj/astar-algorithm-cpp) (for the stlastar.h)
