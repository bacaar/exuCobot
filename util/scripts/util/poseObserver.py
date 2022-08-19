#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 11.08.2022

Just another script for recoring robot movement, this time for pose controller and all 3 axes
"""

import sys

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt

scriptStartTime = None	# variable for storing start time of script
isRecording = False		# flag to enable/disable recording in Callbacks

exudynTargetList = []
controllerTargetList = []
controllerTrajectoryList = []
currentPoseList = []


## define callback functions

def exudynTargetCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		exudynTargetList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])

def controllerTargetCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		controllerTargetList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])

def controllerTrajectoryCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		controllerTrajectoryList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])


def currentPoseCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		currentPoseList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])

	
# analyze time steps inbetween data points
def analyzeTimeSteps(data):
	dtlist = np.zeros(shape=(data.shape[0]-1, ))
	for i in range(len(data)-1):
		dt = data[i+1] - data[i]
		dtlist[i] = dt
	print("Timesteps: datasize: {}  min: {:.8f}  max: {:.8f}  mean: {:.8f}  std: {:.8f}".format(dtlist.shape[0], np.amin(dtlist), np.amax(dtlist), np.mean(dtlist), np.std(dtlist)))


def main(t):

	rospy.init_node('poseObserver', anonymous=True)
	global scriptStartTime
	global isRecording
	scriptStartTime = rospy.Time.now()

	# setup subscirbers
	rospy.Subscriber("/my_cartesian_pose_controller/setTargetPose", PoseStamped, exudynTargetCallback)			# target set by exudyn
	rospy.Subscriber("/my_cartesian_pose_controller/getCurrentTarget", PoseStamped, controllerTargetCallback)	# target registered by controller
	rospy.Subscriber("/my_cartesian_pose_controller/getEvaluatedTrajectory", PoseStamped, controllerTrajectoryCallback)	# evaluated trajectory by controller
	rospy.Subscriber("/my_cartesian_pose_controller/getCurrentPose", PoseStamped, currentPoseCallback)			# current position of robot controller

	# record data from topics for duration of time
	print("Starting recording for {} seconds".format(t))
	t0 = rospy.Time.now()
	isRecording = True
	rospy.sleep(t)
	isRecording = False
	print("Finished recording")
	print("Exudyn Target length: {}\t\tController target length: {}\t\tCurrent pose length: {}".format(len(exudynTargetList),
																						 			   len(controllerTargetList),
																						 			   len(currentPoseList)))
	
	# convert lists to np.arrays for better handling
	exudynTarget = np.array(exudynTargetList)
	controllerTarget = np.array(controllerTargetList)
	plannedTrajectory = np.array(controllerTrajectoryList)
	currentPose = np.array(currentPoseList)

	# analyze time steps inbetween data points
	for data in (exudynTarget[:,0], controllerTarget[:,0]):
		analyzeTimeSteps(data)

	np.save("exudynTarget", exudynTarget)
	np.save("controllerTarget", controllerTarget)
	np.savetxt("controllerTarget_pos", controllerTarget[:,2], delimiter=", ")
	np.savetxt("controllerTarget_t", controllerTarget[:,0], delimiter=", ")

	# plot trajectories
	fig, ax = plt.subplots(1, 1)

	# plot time dependend
	t = exudynTarget[:,0]
	x = exudynTarget[:,1]
	y = exudynTarget[:,2]
	z = exudynTarget[:,3]
	#ax.plot(t, x, "bx", label="exu target x")
	ax.plot(t, y, "bx", label="sent by exudyn")
	#ax.plot(t, z, "gx", label="exu target z")

	tTarget = controllerTarget[:,0]
	x = controllerTarget[:,1]
	yTarget = controllerTarget[:,2]
	z = controllerTarget[:,3]
	#ax.plot(t, x, "b--", label="controller target x")
	ax.plot(tTarget, yTarget, "r.", label="received by controller")
	#ax.plot(t, z, "g--", label="controller target z")

	axsecondary = ax.twinx()
	t = plannedTrajectory[:,0]
	x = plannedTrajectory[:,1]
	y = plannedTrajectory[:,2]
	z = plannedTrajectory[:,3]
	#ax.plot(t, x, "b-", label="planned trajectory x")
	axsecondary.plot(t, y, "r*", label="interpolated trajectory")
	#axsecondary.plot(tTarget+0.25, yTarget, "k.", label="target")
	#ax.plot(t, z, "g-", label="planned trajectory z")

	t = currentPose[:,0]
	x = currentPose[:,1]
	y = currentPose[:,2]
	z = currentPose[:,3]
	#ax.plot(t, x, "b-.", label="measured x")
	#ax.plot(t, y, "r-.", label="measured y")
	#ax.plot(t, z, "g-.", label="measured z")

	ax.set_ylim([-0.15, 0.15])
	axsecondary.set_ylim([-0.15, 0.15])

	ax.grid()
	ax.set_xlabel('t in s')
	ax.set_ylabel('position in m')
	
	axsecondary.legend(loc="upper right")
	ax.legend(loc="upper left")
	
	plt.show()

if __name__ == "__main__":
	try:

		t = 2	# duration of planned recording in seconds

		# parse arguments
		i = 0
		while i < len(sys.argv):
			arg = sys.argv[i]
			if i == 0:
				pass	# program name is no for us relevant parameter

			else:
				t = float(arg)

			i += 1

		main(t)

	except rospy.ROSInterruptException:
		pass
