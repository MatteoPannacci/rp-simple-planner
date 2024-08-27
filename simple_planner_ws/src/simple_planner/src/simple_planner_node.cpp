#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "planner.h"
#include "grid_map/grid_map.h"


int main(int argc, char** argv) {


    // INITIALIZATION //

    // reading arguments
    int wall_cost, wall_cost_decay, step_cost;
    if(argc == 1) {
        wall_cost = 10;
        wall_cost_decay = 1;
        step_cost = 1;
    }
    else if(argc == 4) {
        try {
            wall_cost = std::stoi(std::string(argv[1]));
            wall_cost_decay = std::stoi(std::string(argv[2]));
            step_cost = std::stoi(std::string(argv[3]));
            assert(wall_cost >= 0 || wall_cost_decay > 0 || step_cost >= 0);
        }
        catch(...) {
            std::cout << "invalid arguments provided: wall_cost and step_cost"
                         "should be a non-negative integer while wall_cost_decay"
                         "should be a striclty positive integer" << std::endl;
            return 0;
        }
    }
    else {
        std::cout << "follow this format: 'simple_planner node <wall_cost> "
                     "<wall_cost_decay> <step_cost>'." << std::endl;
        std::cout << "otherwise don't specify parameters to use the default values:" 
                     "wall_cost=10, wall_cost_decay=1, step_cost=1" << std::endl;
        return 0;
    }

    // initialize the node
    ros::init(argc, argv, "simple_planner");

    // set handle
    ros::NodeHandle nh;
    std::string map_topic = "/map";
    std::string goal_topic = "/move_base_simple/goal";
    std::string start_topic = "/tf";
    std::string target = "base_link";
    std::string source = "odom";


    // READING INPUT MESSAGES //

    // read the map
    nav_msgs::OccupancyGrid occupancyGrid;
    std::cout << "-- waiting for map --" << std::endl;
    auto sharedPtrOccupancyGrid = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh);
    if(sharedPtrOccupancyGrid == NULL) {
        std::cout << "-- no map received --" << std::endl;
        std::cout << "-- terminating --" << std::endl;
        return 0;
    }
    else {
        occupancyGrid = *sharedPtrOccupancyGrid;
        std::cout << "-- map read --" << std::endl;
    }

    // read the goal position
    geometry_msgs::Point goal_point;
    std::cout << "-- waiting for goal position --" << std::endl;
    auto sharedPtrGoal = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(goal_topic, nh);
    if(sharedPtrGoal == NULL) {
        std::cout << "-- no goal position received --" << std::endl;
        std::cout << "-- terminating --" << std::endl;
        return 0;
    }
    else {
        goal_point = (*sharedPtrGoal).pose.position;
        std::cout << "-- goal position read --" << std::endl;
        std::cout << "goal position: (" << goal_point.x << " " << goal_point.y << ")" << std::endl;
    }

    // read the start position
    geometry_msgs::Point start_point;
    std::cout << "-- waiting for start position --" << std::endl;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    try {
        listener.waitForTransform(target, source, ros::Time(0), ros::Duration(30.0));
        listener.lookupTransform(target, source, ros::Time(0), transform);
    }
    catch(...) {
        std::cout << "-- no start position received --" << std::endl;
        std::cout << "-- terminating --" << std::endl;
        return 0;
    }
    start_point.x = transform.getOrigin().getX();
    start_point.y = transform.getOrigin().getY();
    start_point.z = transform.getOrigin().getZ();
    std::cout << "-- start position read --" << std::endl;
    std::cout << "start position: (" << start_point.x << " " << start_point.y << ")" << std::endl;


    // PLANNING //

    // create planner
    Planner planner;
    planner.set_map(occupancyGrid, wall_cost, wall_cost_decay, step_cost);
    planner.set_start(start_point);
    planner.set_goal(goal_point);

    // find path
    std::cout << "-- starting planning --" << std::endl;
    planner.find_path();
    std::cout << "-- finishing planning --" << std::endl;
    nav_msgs::Path path = planner.get_path();
    if(path.poses.size() == 0) {
        std::cout << "path not found" << std::endl;
    }
    else {
        std::cout << "path found" << std::endl;
    }


    // VISUALIZATION //

    // visualization of the map
    GridMap grid_map;
    grid_map.loadFromOccupancyGrid(occupancyGrid);
    Canvas canvas;
    grid_map.draw(canvas);

    // visualization of start and goal
    Eigen::Vector2f start_vector(start_point.x, start_point.y);
    Eigen::Vector2f goal_vector(goal_point.x, goal_point.y);
    Eigen::Vector2f grid_start = grid_map.world2grid(start_vector);
    Eigen::Vector2f grid_goal = grid_map.world2grid(goal_vector);
    drawFilledCircle(canvas, grid_start, 10, cv::viz::Color::green());
    drawFilledCircle(canvas, grid_goal, 10, cv::viz::Color::red());

    // visualization of the path
    std::vector<Eigen::Vector2f> grid_path;
    for(geometry_msgs::PoseStamped pose_stamped: path.poses) {
        geometry_msgs::Pose pose = pose_stamped.pose;
        Eigen::Vector2f pose_vector(pose.position.x, pose.position.y);
        Eigen::Vector2f grid_pose = grid_map.world2grid(pose_vector);
        grid_path.push_back(grid_pose);
    }
    drawPath(canvas, grid_path, cv::viz::Color::amethyst());

    // create image
    std::cout << "-- visualizing path --" << std::endl;
    showCanvas(canvas, 0);
    
}