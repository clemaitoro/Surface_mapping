#include "ros/ros.h"

// Declare the function from the C file
extern "C" void main_();  // Declaration of the function in test.c

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_ros_node");
    ros::NodeHandle nh;

    ROS_INFO("Calling a function from C code...");
    main_();  // Call the function from test.c

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
