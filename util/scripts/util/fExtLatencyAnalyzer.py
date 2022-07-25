#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 25.07.2022

Script for analyzing where latency between force-input and motion comes from
"""

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped

import numpy as np

import matplotlib.pyplot as plt

# provide storage space
forceInputList = []
targetPoseList = []
currentPoseList = []

scriptStartTime = None
isRecording = False


## define callback functions
def externalForceCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        
        # get forces
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z

        forceInputList.append([t, fy, fz, fx])


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


def main():
    rospy.init_node('fExtLatencyAnalyzer', anonymous=True)
    global scriptStartTime
    global isRecording
    scriptStartTime = rospy.Time.now()

    # duration of recording
    duration = 2

	# setup subscirbers
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalForceCallback)   
    rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)
    # record data from topics for duration of time
    print("Starting recording for {} seconds".format(duration))
    isRecording = True
    rospy.sleep(duration)
    isRecording = False
    print("Finished recording")

    # plot trajectories
    plt.figure()

    forceInput = np.array(forceInputList)
    targetPose = np.array(targetPoseList)
    currentPose = np.array(currentPoseList)

    # plot x, y and z axes in different subplots
    for i_, ax in enumerate(["x", "y", "z"]):
        i = i_ + 1
        plt.subplot(1, 3, i)
        plt.plot(forceInput[:, 0], forceInput[:, i])
        plt.plot(targetPose[:, 0], targetPose[:, i])
        plt.plot(currentPose[:, 0], currentPose[:, i])

        plt.xlabel("t in s")
        plt.ylabel(ax + "-axis in m")
        plt.grid()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()