#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>


nav_msgs::OccupancyGrid map;


void callback(const geometry_msgs::PoseStamped& goal) {
    std::cout << "-- position read --" << std::endl;
}


int main(int argc, char** argv) {

    // initialize the node
    ros::init(argc, argv, "simple_planner");

    // set handle
    ros::NodeHandle nh;
    std::string map_topic = "/map";
    std::string goal_topic = "/move_base_simple/goal";

    // read the map (only once)
    std::cout << "-- waiting for map --" << std::endl;
    auto sharedPtrMap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh);
    if(sharedPtrMap == NULL) {
        std::cout << "-- no map received --" << std::endl;
        std::cout << "-- terminating --" << std::endl;
        return 0;
    }
    else {
        map = *sharedPtrMap;
        std::cout << "-- map read --" << std::endl;
    }

    // subscribe to the goal topic
    ros::Subscriber sub = nh.subscribe<const geometry_msgs::PoseStamped&>(goal_topic, 5, callback);

    // wait for messages
    ros::spin();

}