#include <ros/ros.h>
#include <GoSdk/GoSdk.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)
#define INVALID_RANGE_16BIT ((signed short)0x8000)

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node1");
    ros::NodeHandle nh;
    ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("gocator_coordinates", 1);
    
    kAssembly api = kNULL;
    kStatus status;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet dataset = kNULL;

    // Initialize Gocator API
    if ((status = GoSdk_Construct(&api)) != kOK) {
        ROS_ERROR("Failed to construct GoSdk: %d", status);
        return 1;
    }

    // Initialize Gocator System
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK) {
        ROS_ERROR("Failed to construct GoSystem: %d", status);
        return 1;
    }

    // Connect to the sensor (you need to set the sensor IP as required)
    kIpAddress ipAddress;
    kIpAddress_Parse(&ipAddress, "10.103.1.150"); // Change to your sensor's IP
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK) {
        ROS_ERROR("Failed to find sensor: %d", status);
        return 1;
    }

    if ((status = GoSensor_Connect(sensor)) != kOK) {
        ROS_ERROR("Failed to connect to sensor: %d", status);
        return 1;
    }

    // Enable data collection
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK) {
        ROS_ERROR("Failed to enable data collection: %d", status);
        return 1;
    }

    // Start the data collection
    if ((status = GoSystem_Start(system)) != kOK) {
        ROS_ERROR("Failed to start system: %d", status);
        return 1;
    }

    while (ros::ok()) {
        // Receive data
        if (GoSystem_ReceiveData(system, &dataset, 20000000) == kOK) { // Adjust the timeout as needed
            for (int i = 0; i < GoDataSet_Count(dataset); ++i) {
                GoDataMsg dataObj = GoDataSet_At(dataset, i);

                if (GoDataMsg_Type(dataObj) == GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD) {
                    GoProfileMsg profileMsg = dataObj;
                    std::vector<float> raw_coordinates;

                    // Fill the raw coordinates using profile message
                    kPoint16s* data = GoProfileMsg_At(profileMsg, 0); // Get the first profile data
                    double XResolution = NM_TO_MM(GoProfileMsg_XResolution(profileMsg));
                    double ZResolution = NM_TO_MM(GoProfileMsg_ZResolution(profileMsg));
                    double XOffset = UM_TO_MM(GoProfileMsg_XOffset(profileMsg));
                    double ZOffset = UM_TO_MM(GoProfileMsg_ZOffset(profileMsg));

                    for (size_t j = 0; j < GoProfileMsg_Width(profileMsg); ++j) {
                        float z = ZOffset + ZResolution * data[j].y;
                        if (data[j].x != INVALID_RANGE_16BIT && z > 1) {
                            float x = XOffset + XResolution * data[j].x;
                            raw_coordinates.push_back(x);
                            raw_coordinates.push_back(z);
                        }
                    }

                    // Publish each point with a timestamp
                    ros::Time current_time = ros::Time::now();
                    for (size_t i = 0; i < raw_coordinates.size(); i += 2) {
                        geometry_msgs::PointStamped point_msg;
                        point_msg.header.stamp = current_time; // Set the timestamp
                        point_msg.header.frame_id = "gocator_frame"; // Set a frame ID if desired
                        point_msg.point.x = raw_coordinates[i];
                        point_msg.point.y = 0.0; // Set y to 0 as profile only provides x and z
                        point_msg.point.z = raw_coordinates[i + 1];
                        coordinates_pub.publish(point_msg);
                    }
                }
            }
            GoDestroy(dataset);
        }

        ros::spinOnce();
    }

    // Clean up
    GoSystem_Stop(system);
    GoDestroy(system);
    GoDestroy(api);
    return 0;
}
