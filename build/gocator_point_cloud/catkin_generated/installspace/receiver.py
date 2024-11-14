#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped, PointStamped
import std_msgs.msg
import numpy as np
import tf.transformations
import csv
from collections import deque
import subprocess  # Import subprocess for starting processes
import time

# Deque to store TCP poses with a max length
tcp_poses = deque(maxlen=100000)
tcp_node_process = None
simple_node_process = None
first_scan_complete = False
second_scan_started = False  # Track if the first scan is complete
x_scan_active = False  # Track if the x-axis scan is active

# Lists to store points from each scan
y_axis_scan_points = []
x_axis_scan_points = []  # Store TCP points during x-axis scan
y_axis_scan_sensor_points = []  # Store unadjusted sensor points during the y-axis scan
x_axis_scan_sensor_points = []  # Store sensor points during the x-axis scan


def start_tcp_node():
    global tcp_node_process
    if tcp_node_process is None:
        tcp_node_process = subprocess.Popen(['rosrun', 'gocator_point_cloud', 'modbus_coordiantes.py'])
        rospy.loginfo("Started TCP node.")


def stop_tcp_node():
    global tcp_node_process
    if tcp_node_process:
        tcp_node_process.terminate()
        tcp_node_process.wait()
        tcp_node_process = None
        rospy.loginfo("Stopped TCP node.")


def start_simple_node():
    global simple_node_process
    
    simple_node_process = subprocess.Popen(['rosrun', 'your_package_name', 'test_node1'])
    rospy.loginfo("Started simple_node.")


def stop_simple_node():
    global simple_node_process
    if simple_node_process:
        try:
            # Attempt to kill the ROS node using rosnode kill
            rospy.loginfo("Attempting to kill the simple_node using rosnode kill...")
            subprocess.run(['rosnode', 'kill', '/test_node1'], check=True)
            simple_node_process = None

        except subprocess.CalledProcessError as e:
            pass

rz_values = deque(maxlen=10)  # Store the last 10 values of rz

def tcp_pose_callback(msg): # TransformStamped
    """Callback for the /tcp_position topic."""
    global first_scan_complete, rz_values, second_scan_started, x_scan_active

    rz = msg.transform.rotation.z
    x = msg.transform.translation.x
    rospy.loginfo(f"tcp_pose_callback - Received message: rz={msg.transform.rotation.z}, x={msg.transform.translation.x}, 1stscan={first_scan_complete}")

    # Append TCP poses
    tcp_poses.append({
        'timestamp': msg.header.stamp,
        'pose': {
            'x': msg.transform.translation.x,
            'y': msg.transform.translation.y,
            'z': msg.transform.translation.z,
            'rx': msg.transform.rotation.x,
            'ry': msg.transform.rotation.y,
            'rz': msg.transform.rotation.z
        }
    })

    # Store TCP points during x-axis scan
    if x_scan_active:
        x_axis_scan_points.append(msg.transform.translation.x)

    # Condition to stop the first scan when rz becomes positive (First scan on y-axis)
    if not first_scan_complete and rz < -0.01:
        rospy.loginfo(f"rz became negative: {rz}, stopping simple_node (First Scan Complete).")
        stop_simple_node()
        first_scan_complete = True

    # After the first scan is complete, check for rz stability to start the second scan
    if first_scan_complete and not second_scan_started:
        rz_values.append(rz)

        # If we have enough values in rz_values, check if rz is stable
        if len(rz_values) == rz_values.maxlen:
            rz_std = np.std(rz_values)  # Calculate the standard deviation of the last rz values

            # If the standard deviation is below a threshold, we assume rz is stable
            if not second_scan_started and  abs(rz + 0.6) < 0.1:
                rospy.loginfo(f"rz is stable around -0.6: {rz}, starting simple_node for Second Scan.")
                start_simple_node()
                second_scan_started = True  # Set the flag to indicate the second scan has started
                x_scan_active = True  # Activate x-axis scan
                # Clear the rz_values to avoid re-triggering
                rz_values.clear()

    # Condition to stop the second scan when x reaches -26.9
    if first_scan_complete and second_scan_started and simple_node_process and x < -26.90:
        # rospy.loginfo(f"x reached -26.9: {x}, stopping simple_node (Second Scan Complete).")
        stop_simple_node()
        second_scan_started = False  # Reset the flag after the second scan is complete
        x_scan_active = False  # Deactivate x-axis scan
        adjust_and_publish_combined_pointcloud()


def gocator_callback(point_msg): # PointStamped
    """Callback for the gocator_coordinates topic."""
    rate = rospy.Rate(1000)

    gocator_timestamp_ns = point_msg.header.stamp.to_nsec()
    closest_pose = find_closest_tcp_pose(gocator_timestamp_ns)

    if closest_pose:
        # Apply transformations based on closest TCP pose
        roll, pitch, yaw = closest_pose['pose']['rx'], closest_pose['pose']['ry'], closest_pose['pose']['rz']
        rotation_matrix = tf.transformations.euler_matrix(roll, pitch, yaw, axes='sxyz')[:3, :3]
        translation_vector = np.array([-closest_pose['pose']['x'], closest_pose['pose']['y'], closest_pose['pose']['z']])

        # Transform the point from Gocator data using the TCP pose
        point_vector = np.array([point_msg.point.x, point_msg.point.y, point_msg.point.z])
        point_rotated = np.dot(rotation_matrix, point_vector)
        point_transformed = point_rotated + translation_vector

        # Store points from each scan
        if not first_scan_complete:
            # Store sensor points during y-axis scan
            y_axis_scan_sensor_points.append(point_transformed)
            rospy.loginfo(f"Accumulated {len(y_axis_scan_sensor_points)} points for the y-axis scan.")
        elif x_scan_active:
            # Store sensor points during x-axis scan
            x_axis_scan_sensor_points.append(point_transformed)
            rospy.loginfo(f"Accumulated {len(x_axis_scan_sensor_points)} points for the x-axis scan.")

        # Publish transformed point to the point cloud topic (for visualization during scanning)
        header = std_msgs.msg.Header()
        header.stamp = point_msg.header.stamp  # Use original Gocator timestamp
        header.frame_id = "map"

        point_cloud_msg = pc2.create_cloud_xyz32(header, [point_transformed])
        pub.publish(point_cloud_msg)


def find_closest_tcp_pose(batch_time_ns):
    """Find the two closest TCP poses and interpolate if needed."""
    if not tcp_poses or len(tcp_poses) < 2:
        rospy.logwarn("Not enough TCP poses available for interpolation.")
        return tcp_poses[0] if tcp_poses else None

    before_pose = None
    after_pose = None

    for i in range(1, len(tcp_poses)):
        if int(tcp_poses[i - 1]['timestamp'].to_nsec()) <= batch_time_ns <= int(tcp_poses[i]['timestamp'].to_nsec()):
            before_pose = tcp_poses[i - 1]
            after_pose = tcp_poses[i]
            break

    if before_pose and after_pose:
        time_diff = int(after_pose['timestamp'].to_nsec()) - int(before_pose['timestamp'].to_nsec())
        batch_time_diff = batch_time_ns - int(before_pose['timestamp'].to_nsec())

        if time_diff > 0:
            ratio = batch_time_diff / time_diff
            interpolated_pose = {
                'timestamp': rospy.Time(nsecs=batch_time_ns),
                'pose': {
                    'x': before_pose['pose']['x'] + ratio * (after_pose['pose']['x'] - before_pose['pose']['x']),
                    'y': before_pose['pose']['y'] + ratio * (after_pose['pose']['y'] - before_pose['pose']['y']),
                    'z': before_pose['pose']['z'] + ratio * (after_pose['pose']['z'] - before_pose['pose']['z']),
                    'rx': before_pose['pose']['rx'] + ratio * (after_pose['pose']['rx'] - before_pose['pose']['rx']),
                    'ry': before_pose['pose']['ry'] + ratio * (after_pose['pose']['ry'] - before_pose['pose']['ry']),
                    'rz': before_pose['pose']['rz'] + ratio * (after_pose['pose']['rz'] - before_pose['pose']['rz']),
                }
            }
            return interpolated_pose

    return min(tcp_poses, key=lambda p: abs(int(p['timestamp'].to_nsec()) - batch_time_ns))


def adjust_and_publish_combined_pointcloud():
    """Adjust the y-axis scan points and combine them with x-axis scan points, then publish as a PointCloud2."""
    # Calculate the range of x values from the x-axis scan sensor points
    x_axis_x_values = [point[0] for point in x_axis_scan_sensor_points]
    x_min, x_max = min(x_axis_x_values), max(x_axis_x_values)

    # Adjust y-axis sensor points to align with the sensor x values from x-axis scan
    adjusted_y_axis_points = []
    for point in y_axis_scan_sensor_points:
        sensor_x_value = point[0]
        if x_min != x_max:  # Avoid division by zero
            offset = (sensor_x_value - x_min) / (x_max - x_min)
            adjusted_x = x_min + offset * (x_max - x_min)
            adjusted_point = [adjusted_x, point[1], point[2]]
            adjusted_y_axis_points.append(adjusted_point)

    # Combine the points from both scans
    combined_points = adjusted_y_axis_points + x_axis_scan_sensor_points

    if len(combined_points) == 0:
        rospy.logwarn("No points accumulated to publish combined point cloud.")
        return

    # Publish combined point cloud
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    point_cloud_msg = pc2.create_cloud(header, fields, combined_points)
    combined_pub.publish(point_cloud_msg)

    # Logging or saving data to CSV
    # x is the x-axis coordinate, y is the y-axis coordinate, z is the z-axis coordinate
    with open('combined_points_final.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for point in combined_points:
            writer.writerow([point[0], point[1], point[2]])


def start():
    global pub, combined_pub
    rospy.init_node('point_cloud_receiver', disable_signals=True)

    # Start the TCP node immediately when the script starts
    start_tcp_node()

    # Start simple_node at the beginning for the first scan
    start_simple_node()

    pub = rospy.Publisher('point_cloud_topic', PointCloud2, queue_size=1000)
    combined_pub = rospy.Publisher('combined_pointcloud', PointCloud2, queue_size=1000)

    rospy.Subscriber('/tcp_position', TransformStamped, tcp_pose_callback)
    rospy.Subscriber('gocator_coordinates', PointStamped, gocator_callback)
    rospy.spin()
if __name__ == '__main__':
    start()