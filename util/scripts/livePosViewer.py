#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 09.06.2022

Script for live viewing desired and current poses
"""

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt

targetPosesList = []
currentPosesList = []

maxLength = 100

# plot every xth position only. Therefore counters are needed
x = 100
counter1 = 0
counter2 = 0

## -------------------------------
## define callback functions

def targetPoseCallback(data):
	
	global targetPosesList
	global counter1

	counter1 += 1

	if counter1 >= x:
		targetPosesList.append([data.pose.position.x, data.pose.position.y, data.pose.position.z])
		counter1 = 0
	
	if len(targetPosesList) > maxLength:
		targetPosesList.pop(0)


def currentPoseCallback(data):

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
	rospy.init_node('livePosViewer', anonymous=True)		

	# setup subscirbers
	rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
	rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)	

	while not rospy.is_shutdown():

		targetPoses = np.array(targetPosesList)
		currentPoses = np.array(currentPosesList)

		if targetPoses.shape[0] > 0:
			plt.plot(targetPoses[:, 0], targetPoses[:, 1], 'b', linewidth=0.1)

		if currentPoses.shape[0] > 0:
			plt.plot(currentPoses[:, 0], currentPoses[:, 1], 'r', linewidth=0.1)


		plt.axis('equal')
		plt.draw()
		plt.pause(0.0000001)	# needed for updating and showing window
	
except rospy.ROSInterruptException:
	pass
