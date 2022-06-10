#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 01.06.2022

Script for passing new target points to impedance controller
for testing purposes
"""

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import tf

import time

def main():

	# create correct publisher
	pub = rospy.Publisher('/my_cartesian_impedance_controller/setDesiredPose', PoseStamped, queue_size=10)

	# init rospy, init some variables
	rospy.init_node('newPoses', anonymous=True)
	rate = rospy.Rate(1000)
	#npose = 1

	# for circle creation
	t0 = time.time()
	xc = 0.5
	yc = 0.0
	r = 0.2

	while not rospy.is_shutdown():
		# get time since program start (used as trajectory-parameter)
		t1 = time.time()
		t = t1-t0

		# calculate coordinates for circle
		x = xc + r * np.cos(t)
		y = yc + r * np.sin(t)

		# create message
		msg = PoseStamped()
		
		# write position into message
		msg.pose.position.x = x
		msg.pose.position.y = y
		msg.pose.position.z = 0.3

		# endeffector should point straight down
		pitch = np.radians(180)
		roll = np.radians(0)
		yaw = np.radians(0)

		# create Quaternion out of Euler angles
		quaternion = tf.transformations.quaternion_from_euler(pitch, yaw, roll)

		# only to be sure quaternion is correct
		assert np.linalg.norm(quaternion) == 1.0, "ERROR"

		# write orientation into message
		msg.pose.orientation.x = quaternion[0]
		msg.pose.orientation.y = quaternion[1]
		msg.pose.orientation.z = quaternion[2]
		msg.pose.orientation.w = quaternion[3]

		#print("publish Pose " + str(npose))
		#npose = npose+1

		# write current time into message
		msg.header.stamp = rospy.Time.now()
		print(msg)
		pub.publish(msg)

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
