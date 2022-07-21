#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 09.06.2022

Script for live viewing desired and current poses
"""

from email.errors import InvalidMultipartContentTransferEncodingDefect
import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt

import sys

targetPosesList = []
currentPosesList = []

maxLength = 100

# plot every xth position only. Therefore counters are needed
x = 50
counter1 = 0
counter2 = 0

## -------------------------------
## define callback functions

def targetPoseCallback(data):
	"""
	callback function for desired pose / target pose
	actually we are only interested in the position, not the full pose

	:param geometry_msgs.msg.PoseStamped data: received message
	"""
	
	global targetPosesList
	global counter1

	# as mentioned above, add only each xth pose to plot
	counter1 += 1

	if counter1 >= x:
		targetPosesList.append([data.pose.position.x, data.pose.position.y, data.pose.position.z])
		counter1 = 0
	
	# keep only last maxLength entries in list
	# note: actually don't even needed. I originally wanted to only plot the last few passed positions, but it seems as matplotlib
	# keeps all the points in the plot
	if len(targetPosesList) > maxLength:
		targetPosesList.pop(0)


def currentPoseCallback(data):
	"""
	callback function for current pose / measured pose
	actually we are only interested in the position, not the full pose

	function has same purpose as targetPoseCallback(data). For further and more detailed explanations look there

	:param geometry_msgs.msg.PoseStamped data: received message
	"""

	global currentPosesList
	global counter2

	counter2 += 1

	if counter2 >= x:
		currentPosesList.append([data.pose.position.x, data.pose.position.y, data.pose.position.z])
		counter2 = 0

	if len(currentPosesList) > maxLength:
		currentPosesList.pop(0)

## -------------------------------
## main
try:

	# first user parameter defines plane which has to be depicted
	plane = "yz"	# default value
	if len(sys.argv) >= 2:
		if sys.argv[1] == "xy":
			plane = "xy"
		elif sys.argv[1] == "yz":
			pass	# default value
		else:
			print("unknown plane", end="")
		print(" Using " + plane)


	rospy.init_node('livePosViewer', anonymous=True)		

	# set up subscribers
	rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
	rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)	

	while not rospy.is_shutdown():

		targetPoses = np.array(targetPosesList)
		currentPoses = np.array(currentPosesList)

		if targetPoses.shape[0] > 0:	# range check
			if plane == "xy":
				plt.plot(targetPoses[:, 0], targetPoses[:, 1], 'b', linewidth=0.1)
			if plane == "yz":
				plt.plot(targetPoses[:, 1], targetPoses[:, 2], 'b', linewidth=0.1)

		if currentPoses.shape[0] > 0:
			if plane == "xy":
				plt.plot(currentPoses[:, 0], currentPoses[:, 1], 'r', linewidth=0.1)
			if plane == "yz":
				plt.plot(currentPoses[:, 1], currentPoses[:, 2], 'r', linewidth=0.1)

		plt.axis('equal')
		plt.draw()
		plt.pause(0.0000001)	# needed for updating and showing window
	
except rospy.ROSInterruptException:
	pass
