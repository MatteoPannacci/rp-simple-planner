#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv) {

    double x,y,rate;

    // read input
    if(argc < 4) {
        std::cout << "follow this format: goal_publisher_node <x> <y> <rate>" << std::endl;
        return 0;
    }
    else {
        try {
            x = std::stod(std::string(argv[1]));
            y = std::stod(std::string(argv[2]));
            rate = std::stod(std::string(argv[3]));
            assert(rate >= 0);
        }
        catch(...) {
            std::cout << "invalid arguments provided: x and y should be floating"
                         "point numbers while rate should be a positive floating"
                         "point number, or 0 to send the message once" << std::endl;
            return 0;
        }

    }

    // initialize the node
    ros::init(argc, argv, "goal_publisher");

    // set handle
    ros::NodeHandle nh;
    std::string goal_topic = "/move_base_simple/goal";

    // publish the goal topic
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 5);

    // create the message
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "/map";
    msg.header.seq = 0;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1;


    if(rate ==  0) {
        // send the message once
        pub.publish(msg);
        ros::spinOnce();
        std::cout << "-- message published --" << std::endl;
    }

    else {
        // send the message at the specified rate
        ros::Rate loop_rate(rate);
        while(ros::ok()) {
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            std::cout << "-- message n. " << std::to_string(msg.header.seq) << " published --" << std::endl;
            msg.header.seq += 1;
        }
    }

    std::cout << "-- terminating --" << std::endl;

}