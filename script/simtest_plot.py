#!/usr/bin/env python3

import sys
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock


# Variables
time = []

cmdvel_time = []
cmdvel_linvel = []
cmdvel_angvel = []

odom_time = []
odom_x = []
odom_y = []
odom_z = []
odom_qx = []
odom_qy = []
odom_qz = []
odom_qw = []
odom_roll = []
odom_pitch = []
odom_yaw = []
odom_linvel = []
odom_angvel = []

vicon_localization_odom_time = []
vicon_localization_odom_x = []
vicon_localization_odom_y = []
vicon_localization_odom_z = []
vicon_localization_odom_qx = []
vicon_localization_odom_qy = []
vicon_localization_odom_qz = []
vicon_localization_odom_qw = []
vicon_localization_odom_roll = []
vicon_localization_odom_pitch = []
vicon_localization_odom_yaw = []
vicon_localization_odom_linvel = []
vicon_localization_odom_angvel = []

vicon_localization_pose_time = []
vicon_localization_pose_x = []
vicon_localization_pose_y = []
vicon_localization_pose_z = []
vicon_localization_pose_qx = []
vicon_localization_pose_qy = []
vicon_localization_pose_qz = []
vicon_localization_pose_qw = []
vicon_localization_pose_roll = []
vicon_localization_pose_pitch = []
vicon_localization_pose_yaw = []


# Open the bag
storage_options = rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap')
converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
        
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)
        
topic_types = reader.get_all_topics_and_types()
        
# Create a map for quicker lookup
type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

# Sequentially read the bag
while reader.has_next():
	(topic, data, t) = reader.read_next()
	msg_type = get_message(type_map[topic])
	msg = deserialize_message(data, msg_type)

	# Extract clock data
	if isinstance(msg, Clock) and (topic=='/clock'):
		# Extract data from message and store it in temporary arrays
		time.append(msg.clock.sec+msg.clock.nanosec*1e-9)

	# Extract cmd_vel data
	if isinstance(msg, Twist) and (topic=='/cmd_vel'):
		# Store last time value
		if not time:
			continue
		cmdvel_time.append(time[-1])

		# Extract data from message and store it in temporary arrays
		cmdvel_linvel.append(msg.linear.x)
		cmdvel_angvel.append(msg.angular.z)

	# Extract odom data
	if isinstance(msg, Odometry) and (topic=='/odom'):
		# Extract data from message and store it in temporary arrays
		odom_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

		odom_x.append(msg.pose.pose.position.x)
		odom_y.append(msg.pose.pose.position.y)
		odom_z.append(msg.pose.pose.position.z)
		
		odom_qx.append(msg.pose.pose.orientation.x)
		odom_qy.append(msg.pose.pose.orientation.y)
		odom_qz.append(msg.pose.pose.orientation.z)
		odom_qw.append(msg.pose.pose.orientation.w)
		
		(roll, pitch, yaw) = Rotation.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]).as_euler('xyz')
		odom_roll.append(roll)
		odom_pitch.append(pitch)
		odom_yaw.append(yaw)

		odom_linvel.append(msg.twist.twist.linear.x)
		odom_angvel.append(msg.twist.twist.angular.z)

	# Extract vicon_localization_odom data
	if isinstance(msg, Odometry) and (topic=='/vicon_localization_odom'):
		# Extract data from message and store it in temporary arrays
		vicon_localization_odom_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

		vicon_localization_odom_x.append(msg.pose.pose.position.x)
		vicon_localization_odom_y.append(msg.pose.pose.position.y)
		vicon_localization_odom_z.append(msg.pose.pose.position.z)
		
		vicon_localization_odom_qx.append(msg.pose.pose.orientation.x)
		vicon_localization_odom_qy.append(msg.pose.pose.orientation.y)
		vicon_localization_odom_qz.append(msg.pose.pose.orientation.z)
		vicon_localization_odom_qw.append(msg.pose.pose.orientation.w)
		
		(roll, pitch, yaw) = Rotation.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]).as_euler('xyz')
		vicon_localization_odom_roll.append(roll)
		vicon_localization_odom_pitch.append(pitch)
		vicon_localization_odom_yaw.append(yaw)

		vicon_localization_odom_linvel.append(msg.twist.twist.linear.x)
		vicon_localization_odom_angvel.append(msg.twist.twist.angular.z)

	# Extract vicon_localization_pose data
	if isinstance(msg, PoseWithCovarianceStamped) and (topic=='/vicon_localization_pose'):
		# Extract data from message and store it in temporary arrays
		vicon_localization_pose_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

		vicon_localization_pose_x.append(msg.pose.pose.position.x)
		vicon_localization_pose_y.append(msg.pose.pose.position.y)
		vicon_localization_pose_z.append(msg.pose.pose.position.z)
		
		vicon_localization_pose_qx.append(msg.pose.pose.orientation.x)
		vicon_localization_pose_qy.append(msg.pose.pose.orientation.y)
		vicon_localization_pose_qz.append(msg.pose.pose.orientation.z)
		vicon_localization_pose_qw.append(msg.pose.pose.orientation.w)
		
		(roll, pitch, yaw) = Rotation.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]).as_euler('xyz')
		vicon_localization_pose_roll.append(roll)
		vicon_localization_pose_pitch.append(pitch)
		vicon_localization_pose_yaw.append(yaw)

# Unwrap vectors of angles
odom_roll = np.unwrap(odom_roll)
odom_pitch = np.unwrap(odom_pitch)
odom_yaw = np.unwrap(odom_yaw)
vicon_localization_odom_roll = np.unwrap(vicon_localization_odom_roll)
vicon_localization_odom_pitch = np.unwrap(vicon_localization_odom_pitch)
vicon_localization_odom_yaw = np.unwrap(vicon_localization_odom_yaw)
vicon_localization_pose_roll = np.unwrap(vicon_localization_pose_roll)
vicon_localization_pose_pitch = np.unwrap(vicon_localization_pose_pitch)
vicon_localization_pose_yaw = np.unwrap(vicon_localization_pose_yaw)

# Plot data
plt.figure(1)
plt.plot(odom_x, odom_y, label='Ground truth')
plt.plot(vicon_localization_odom_x, vicon_localization_odom_y, 'r--', label='Vicon Localization Odometry')
plt.plot(vicon_localization_pose_x, vicon_localization_pose_y, 'b-.', label='Vicon Localization Pose')
plt.legend()
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.figure(2)
plt.plot(odom_time,odom_x, label='Ground truth')
plt.plot(vicon_localization_odom_time,vicon_localization_odom_x, 'r--', label='Vicon Localization Odometry')
plt.plot(vicon_localization_pose_time,vicon_localization_pose_x, 'b-.', label='Vicon Localization Pose')
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("x [m]")

plt.figure(3)
plt.plot(odom_time,odom_y, label='Ground truth')
plt.plot(vicon_localization_odom_time,vicon_localization_odom_y, 'r--', label='Vicon Localization Odometry')
plt.plot(vicon_localization_pose_time,vicon_localization_pose_y, 'b-.', label='Vicon Localization Pose')
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("y [m]")

plt.figure(4)
plt.plot(odom_time,np.degrees(odom_yaw), label='Ground truth')
plt.plot(vicon_localization_odom_time,np.degrees(vicon_localization_odom_yaw), 'r--', label='Vicon Localization Odometry')
plt.plot(vicon_localization_pose_time,np.degrees(vicon_localization_pose_yaw), 'b-.', label='Vicon Localization Pose')
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Yaw [deg]")

plt.figure(5)
plt.plot(cmdvel_time,cmdvel_linvel, label='Ground truth')
plt.plot(vicon_localization_odom_time,vicon_localization_odom_linvel, 'r--', label='Vicon Localization Odometry')
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity command [m/s]")

plt.figure(6)
plt.plot(cmdvel_time,cmdvel_angvel, label='Ground truth')
plt.plot(vicon_localization_odom_time,vicon_localization_odom_angvel, 'r--', label='Vicon Localization Odometry')
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity command [rad/s]")

plt.show()

		
