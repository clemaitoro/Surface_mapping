#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_ros
import std_msgs.msg
import numpy as np
import tf.transformations
import csv
from collections import deque
import subprocess  # Import subprocess for starting the TCP node process

# Deque to store TCP poses with a max length
tcp_poses = deque(maxlen=100000)
tcp_y_count = 0
tcp_node_process = None
stop_count = 0

# Global variable to store initial offset
initial_timestamp_difference_ns = None

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

def tcp_pose_callback(msg):
    """Callback for the /tcp_position topic."""
    global tcp_y_count
    if -418.8 <= msg.transform.translation.y <= -418.7:
        tcp_y_count += 1
        rospy.loginfo(f"Y in range: {msg.transform.translation.y}, count: {tcp_y_count}")

    if tcp_y_count >= 2:
        rospy.loginfo("Y value reached threshold twice, calling stop_tcp_node()")
        # stop_tcp_node()
    
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

def gocator_callback(point_msg):
    """Callback for the gocator_coordinates topic."""
    rate = rospy.Rate(1000)  

    gocator_timestamp_ns = point_msg.header.stamp.to_nsec()
    closest_pose = find_closest_tcp_pose(gocator_timestamp_ns)

    if closest_pose:
        # Apply transformations based on closest TCP pose
        roll, pitch, yaw = closest_pose['pose']['rx'], closest_pose['pose']['ry'], closest_pose['pose']['rz']
        rotation_matrix = tf.transformations.euler_matrix(roll, pitch, yaw, axes='sxyz')[:3, :3]
        translation_vector = np.array([closest_pose['pose']['x'], closest_pose['pose']['y'], closest_pose['pose']['z']])

        # Transform the point from Gocator data using the TCP pose
        point_vector = np.array([point_msg.point.x, 0, point_msg.point.z])  # Set y to TCP y
        point_rotated = np.dot(rotation_matrix, point_vector)
        point_transformed = point_rotated + translation_vector
        

        # Publish transformed point to the point cloud topic
        header = std_msgs.msg.Header()
        header.stamp = point_msg.header.stamp  # Use original Gocator timestamp
        header.frame_id = "map"

        point_cloud_msg = pc2.create_cloud_xyz32(header, [point_transformed])
        pub.publish(point_cloud_msg)

        # Logging or saving data
        with open('transformed_points_final.csv', 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                point_msg.point.x, point_msg.point.z
            ])

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

def start():
    global pub
    rospy.init_node('point_cloud_receiver', disable_signals=True)
    
    # Start the TCP node immediately when the script starts
    start_tcp_node()

    pub = rospy.Publisher('point_cloud_topic', PointCloud2, queue_size=1000)
    rospy.Subscriber('/tcp_position', TransformStamped, tcp_pose_callback)
    rospy.Subscriber('gocator_coordinates', PointStamped, gocator_callback)

    rospy.spin()

if __name__ == '__main__':
    start()
