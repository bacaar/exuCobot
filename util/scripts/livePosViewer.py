#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 09.06.2022

Script for live viewing desired and current poses
"""

import rospy
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt


## -------------------------------
## define callback functions

def targetPoseCallback(data):
	
	global counter1
	
	if counter1 == 100:
		plt.plot(data.pose.position.x, data.pose.position.y, 'bx')
		plt.draw()
		plt.axis('equal')
		plt.pause(0.0000001)	# needed for updating and showing window
		counter1 = -1

	counter1 += 1


def currentPoseCallback(data):
	
	global counter2
	
	if counter2 == 100:
		plt.plot(data.pose.position.x+0.1, data.pose.position.y, 'rx')
		plt.draw()
		plt.pause(0.0000001)
		counter2 = -1

	counter2 += 1

## -------------------------------
## main

# plot every 100th position only. Therefore counters are needed
counter1 = 0
counter2 = 0

try:
	rospy.init_node('livePosViewer', anonymous=True)		

	# setup subscirbers
	rospy.Subscriber("/my_cartesian_impedance_example_controller/setDesiredPose", PoseStamped, targetPoseCallback)
	rospy.Subscriber("/my_cartesian_impedance_example_controller/getCurrentPose", PoseStamped, currentPoseCallback)	

	rospy.spin()	# keep programm running until ctrl+C
	
except rospy.ROSInterruptException:
	pass
