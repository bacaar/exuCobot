#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 02.06.2022

Script for recording errors of robot following trajectory
"""

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt

scriptStartTime = None	# variable for storing start time of script
isRecording = False		# flag to enable/disable recording in Callbacks

errorList = []
targetPoseList = []
currentPoseList = []


## define callback functions

def targetPoseCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		t = duration.secs + duration.nsecs*1e-9
		targetPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def currentPoseCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		t = duration.secs + duration.nsecs*1e-9
		currentPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def currentErrorCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		t = duration.secs + duration.nsecs*1e-9
		errorList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def main():

	rospy.init_node('errorRecorder', anonymous=True)
	global scriptStartTime
	global isRecording
	scriptStartTime = rospy.Time.now()
	
	# duration of recording; 2*pi is approx one rotation
	duration = 3*2*np.pi	# duration of planned recording in seconds

	# setup subscirbers
	rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
	rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)
	rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentError", PoseStamped, currentErrorCallback)

	# record data from topics for duration of time
	print("Starting recording for {} seconds".format(duration))
	t0 = rospy.Time.now()
	isRecording = True
	rospy.sleep(duration)
	isRecording = False
	print("Finished recording")
	print("Target pose length: {}\t\tCurrent pose length: {}\t\tError length: {}".format(len(targetPoseList),
																						 len(currentPoseList),
																						 len(errorList)))
	
	# convert lists to np.arrays for better handling
	targetPose = np.array(targetPoseList)
	currentPose = np.array(currentPoseList)
	error = np.array(errorList)

	# plot trajectories
	plt.figure()
	x = targetPose[:,1]
	y = targetPose[:,2]
	plt.plot(x, y, label="desired")
	
	x = currentPose[:,1]
	y = currentPose[:,2]
	plt.plot(x, y, label="measured")
	
	# "global" referes to the robot base coordinate system
	plt.xlabel('global x in m')
	plt.ylabel('global y in m')
	plt.legend(loc="upper right")
	plt.gca().set_aspect('equal', adjustable='box')
	plt.show()


	# plot time dependend
	plt.figure()

	t = targetPose[:,0]
	x = targetPose[:,1]
	y = targetPose[:,2]
	z = targetPose[:,3]
	plt.plot(t, x, "b-", label="target x")
	plt.plot(t, y, "r-", label="target y")
	plt.plot(t, z, "g-", label="target z")

	t = currentPose[:,0]
	x = currentPose[:,1]
	y = currentPose[:,2]
	z = currentPose[:,3]
	plt.plot(t, x, "b-.", label="measured x")
	plt.plot(t, y, "r-.", label="measured y")
	plt.plot(t, z, "g-.", label="measured z")

	#t = error[:,0]
	#x = error[:,1]
	#y = error[:,2]
	#z = error[:,3]
	#plt.plot(t, x, "b:", label="error x")
	#plt.plot(t, y, "r:", label="error y")
	#plt.plot(t, z, "g:", label="error z")

	plt.grid()
	plt.xlabel('t in s')
	plt.ylabel('position in m')
	plt.legend(loc="upper right")
	plt.show()


	# plot time dependend errors only
	plt.figure()

	t = error[:,0]
	x = error[:,1]
	y = error[:,2]
	z = error[:,3]
	abs = np.sqrt(x**2 + y**2 + z**2)

	plt.plot(t, x, "b", label="error x")
	plt.plot(t, y, "r", label="error y")
	plt.plot(t, z, "g", label="error z")
	plt.plot(t, abs, "k", label="error abs")

	plt.grid()
	plt.xlabel('t in s')
	plt.ylabel('position in m')
	plt.legend(loc="upper right")
	plt.show()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
