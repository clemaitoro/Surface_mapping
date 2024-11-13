#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
from pymodbus.client import ModbusTcpClient
import time
import tf.transformations

# Function to read a 16-bit signed integer value from one register
def read_signed_16bit(client, address, unit=1):
    response = client.read_input_registers(address, 1, unit=unit)
    if response.isError():
        rospy.logerr(f"Error reading address {address}")
        return None
    else:
        # Interpret the value as a signed 16-bit integer
        raw_value = response.registers[0]
        if raw_value >= 32768:  # Convert unsigned to signed
            raw_value -= 65536
        return raw_value

# Function to read TCP position and orientation from the UR5e robot
def read_tcp_position_orientation(client, unit=1):
    addresses = {
        'x': 400,
        'y': 401,
        'z': 402,
        'rx': 403,  # Roll in milliradians
        'ry': 404,  # Pitch in milliradians
        'rz': 405   # Yaw in milliradians
    }

    tcp_data = {}
    for axis in ['x', 'y', 'z']:
        position_raw = read_signed_16bit(client, addresses[axis], unit)
        if position_raw is not None:
            tcp_data[axis] = position_raw / 10.0  # Convert millimeters to meters
        else:
            return None

    for axis in ['rx', 'ry', 'rz']:
        orientation_mrad = read_signed_16bit(client, addresses[axis], unit)
        if orientation_mrad is not None:
            tcp_data[axis] = orientation_mrad / 1000  # Convert 
        else:
            return None

    return tcp_data



def publish_tcp_data(client, pub, rate):
    """
    Reads the TCP data from the robot and publishes it to the ROS topic only when the robot is moving.
    """
    last_tcp_data = read_tcp_position_orientation(client)
    if last_tcp_data is None:
        rospy.logerr("Failed to read initial TCP data.")
        return

    rospy.loginfo("Monitoring robot motion...")

    while not rospy.is_shutdown():
        # Read the current TCP data
        current_tcp_data = read_tcp_position_orientation(client)

        if current_tcp_data:
            # if is_robot_moving(current_tcp_data, last_tcp_data):
                # Robot is moving, publish the TCP data
                current_time = time.time()  # Get system time
                ros_time = rospy.Time.from_sec(current_time)

                transform = TransformStamped()
                transform.header.stamp = ros_time
                transform.header.frame_id = "base_link"  # The frame attached to the robot's base
                transform.child_frame_id = "tcp_frame"   # The frame representing the TCP position

                transform.transform.translation.x = current_tcp_data['x']
                transform.transform.translation.y = current_tcp_data['y']
                transform.transform.translation.z = current_tcp_data['z']

                quaternion = tf.transformations.quaternion_from_euler(
                    current_tcp_data['rx'], current_tcp_data['ry'], current_tcp_data['rz'], axes='sxyz'
                )
                transform.transform.rotation.x = quaternion[0]
                transform.transform.rotation.y = quaternion[1]
                transform.transform.rotation.z = quaternion[2]
                transform.transform.rotation.w = quaternion[3]

                pub.publish(transform)
                print(f"Published moving TCP Data: {current_tcp_data} {ros_time}")
            # else:
                # rospy.loginfo("Robot is stationary. No data published.")
                # pass

            # Update last_tcp_data for the next iteration
        last_tcp_data = current_tcp_data

        rate.sleep()

def main():
    # Initialize the ROS node
    rospy.init_node('tcp_position_publisher', anonymous=True)

    # Create a publisher to publish the TransformStamped message
    pub = rospy.Publisher('/tcp_position', TransformStamped, queue_size=10)

    # Define the publish rate (e.g., 10 Hz)
    rate = rospy.Rate(50)  # 10 Hz

    # Modbus TCP client setup
    robot_ip = '10.103.1.229'  # Replace with the actual IP address of the robot
    port = 502  # Default Modbus TCP port
    client = ModbusTcpClient(robot_ip, port)

    try:
        client.connect()
        rospy.loginfo(f'Connected to {robot_ip}:{port}')

        # Start monitoring and publishing TCP data based on motion
        publish_tcp_data(client, pub, rate)

    except Exception as e:
        rospy.logerr(f'An error occurred: {e}')
    
    finally:
        client.close()
        rospy.loginfo('Connection closed.')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
