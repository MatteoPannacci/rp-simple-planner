#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "planner.h"
#include "grid_map/grid_map.h"


nav_msgs::OccupancyGrid occupancyGrid;
geometry_msgs::Point goal;
geometry_msgs::Point start;
nav_msgs::Path path;


int main(int argc, char** argv) {

    // initialize the node
    ros::init(argc, argv, "simple_planner");

    // set handle
    ros::NodeHandle nh;
    std::string map_topic = "/map";
    std::string goal_topic = "/move_base_simple/goal";
    std::string start_topic = "/tf";
    std::string target = "base_link";
    std::string source = "odom";


    // read the map
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

    // read the goal
    std::cout << "-- waiting for goal --" << std::endl;
    auto sharedPtrGoal = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(goal_topic, nh);
    if(sharedPtrGoal == NULL) {
        std::cout << "-- no goal received --" << std::endl;
        std::cout << "-- terminating --" << std::endl;
        return 0;
    }
    else {
        goal = (*sharedPtrGoal).pose.position;
        std::cout << "-- goal read --" << std::endl;
        std::cout << goal << std::endl;
    }

    // read the start
    std::cout << "-- waiting for start --" << std::endl;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    try {
        listener.waitForTransform(target, source, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform(target, source, ros::Time(0), transform);
    }
    catch(...) {
        std::cout << "-- no start received --" << std::endl;
        std::cout << "-- terminating --" << std::endl;
        return 0;
    }
    start = geometry_msgs::Point();
    start.x = transform.getOrigin().getX();
    start.y = transform.getOrigin().getY();
    start.z = transform.getOrigin().getZ();
    std::cout << "-- start read --" << std::endl;
    std::cout << start << std::endl;


    std::cout << "-- start creating map --" << std::endl;
    CostMap costmap(occupancyGrid, 10);
    std::cout << "-- finish creating map --" << std::endl;


    // visualization of the map
    GridMap grid_map;
    grid_map.loadFromOccupancyGrid(occupancyGrid);
    Canvas canvas;
    grid_map.draw(canvas);

    // visualization of start and goal
    Vector2f grid_start = grid_map.world2grid(Vector2f(start.x, start.y));
    Vector2f grid_goal = grid_map.world2grid(Vector2f(goal.x, goal.y));
    drawFilledCircle(canvas, grid_start, 10, cv::viz::Color::green());
    drawFilledCircle(canvas, grid_goal, 10, cv::viz::Color::red());

    // visualization of the path
    std::vector<Vector2f> grid_path;
    for(geometry_msgs::PoseStamped pose_stamped: path.poses) {
        geometry_msgs::Pose pose = pose_stamped.pose;
        Vector2f grid_pose = grid_map.world2grid(Vector2f(pose.position.x, pose.position.y));
    }
    drawPath(canvas, grid_path, cv::viz::Color::amethyst());

    showCanvas(canvas, 0);
    
}