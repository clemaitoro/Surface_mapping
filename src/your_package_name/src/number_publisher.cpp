#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "number_publisher");
    ros::NodeHandle nh;

    // Create a publisher that publishes to the "number_topic"
    ros::Publisher number_pub = nh.advertise<std_msgs::Int32>("number_topic", 1);

    // Set the loop rate (10 Hz)
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // Create a message of type Int32
        std_msgs::Int32 msg;
        msg.data = 1; // Set the value to 1

        // Publish the message
        number_pub.publish(msg);

        // Log the published number
        ROS_INFO("Published: %d", msg.data);

        // Sleep to maintain loop rate
        loop_rate.sleep();
    }

    return 0;
}
