#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 01.08.2022

Script for recording published trajectory and differentiating it two times numerically
"""

import sys

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np

import matplotlib.pyplot as plt


scriptStartTime = None	# variable for storing start time of script
isRecording = False		# flag to enable/disable recording in Callbacks

targetPoseList = []


## define callback function
def targetPoseCallback(data):
	if isRecording:
		duration = data.header.stamp - scriptStartTime
		t = duration.secs + duration.nsecs*1e-9
		targetPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def main(t):
    rospy.init_node('errorRecorder', anonymous=True)
    global scriptStartTime
    global isRecording
    scriptStartTime = rospy.Time.now()

    # setup subscirbers
    rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)

    # record data from topics for duration of time
    print("Starting recording for {} seconds".format(t))
    isRecording = True
    rospy.sleep(t)
    isRecording = False
    dataSize = len(targetPoseList)
    print("Finished recording. Got {} datapoints".format(dataSize))

    # convert lists to np.arrays for better handling
    pos = np.array(targetPoseList)

    # provide some storage space
    vel = np.empty(shape=(dataSize, 4))
    acc = np.empty(shape=(dataSize, 4))

    for i in range(1, dataSize):
        dt = pos[i][0] - pos[i-1][0]

        # numerical differentiation to calulate velocity
        vel[i, 0] = pos[i, 0]
        vel[i, 1:4] = (pos[i][1:4] - pos[i-1][1:4])/dt

        if i > 1:
            # numerical differentiation to calulate acceleration
            acc[i, 0] = pos[i, 0]
            acc[i] = (vel[i] - vel[i-1])/dt

    # create plots
    fig, axs = plt.subplots(3,1,sharex=True)

    fig.suptitle("Pose example controller trajectory analysis")

    # plot absolute position
    axs[0].plot(pos[:,0], pos[:,1], label="x")
    axs[0].plot(pos[:,0], pos[:,2], label="y")
    axs[0].plot(pos[:,0], pos[:,3], label="z")
    axs[0].set_ylabel('position in m')
    axs[0].grid()
    axs[0].legend()

    # plot velocity
    axs[1].plot(pos[1:,0], vel[1:,1], label="x")
    axs[1].plot(pos[1:,0], vel[1:,2], label="y")
    axs[1].plot(pos[1:,0], vel[1:,3], label="z")
    axs[1].set_ylabel('velocity in m/s')
    axs[1].grid()
    axs[1].legend()

    # plot acceleration
    axs[2].plot(pos[2:,0], acc[2:,1], label="x")
    axs[2].plot(pos[2:,0], acc[2:,2], label="y")
    axs[2].plot(pos[2:,0], acc[2:,3], label="z")
    axs[2].set_ylabel('acceleration in m/s2')
    axs[2].grid()
    axs[2].legend()


    axs[2].set_xlabel('time in s')

    plt.show()


if __name__ == "__main__":
    try:

        # default duration of recording in seconds
        t = 3

        # parse arguments
        i = 0
        while i < len(sys.argv):
            arg = sys.argv[i]
            if i == 0:
                pass    # program name is no for us relevant parameter
            elif i > 1:
                break   # only one argument is accepted
            else:
                t = float(arg)

            i += 1

        main(t)

    except rospy.ROSInterruptException:
        pass