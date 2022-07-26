#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 22.07.2022

Script for planning and recording step Response of Robot
"""
import sys

from matplotlib import use

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt

import math

from common import createPoseStampedMsg

t0 = 0
isRecording = False

targetPoseList = []
currentTargetList = []
currentPoseList = []

# define callback functions
def targetPoseCallback(data):
    if isRecording:
        duration = data.header.stamp - t0
        t = duration.secs + duration.nsecs*1e-9
        targetPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])

def currentTargetCallback(data):
    if isRecording:
        duration = data.header.stamp - t0
        t = duration.secs + duration.nsecs*1e-9
        currentTargetList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def currentPoseCallback(data):
    if isRecording:
        duration = data.header.stamp - t0
        t = duration.secs + duration.nsecs*1e-9
        currentPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def trajectoryStepResponse(t_):
    t = math.fmod(t_, 10)

    if t <= 5:
        x = 0.4
        y = 0.1
        z = 0.2
    else:
        x = 0.5
        y = 0.1
        z = 0.2

    return x, y, z


def main(usePoseController):

    # init rospy, init some variables
    rospy.init_node('newPoses', anonymous=True)
    rate = rospy.Rate(1000)

    global t0
    global isRecording

    # create correct publisher
    if usePoseController:
        pub = rospy.Publisher('/my_cartesian_pose_controller/setDesiredPose', PoseStamped, queue_size=10)
    else:
        pub = rospy.Publisher('/my_cartesian_impedance_controller/setDesiredPose', PoseStamped, queue_size=10)

    # setup subscirbers
    rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentTarget", PoseStamped, currentTargetCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)

    t0 = rospy.Time.now()
    isRecording = True
    while not rospy.is_shutdown():
        # get time since program start (used as trajectory-parameter)
        t1 = rospy.Time.now()
        t = (t1-t0).to_sec()

        coords = trajectoryStepResponse(t)

        # create message
        msg = createPoseStampedMsg([coords[0], coords[1], coords[2]], [180, 0, 0], t1)
        pub.publish(msg)

        rate.sleep()
        if t > 20:
            break

    isRecording = False

    # convert lists to np.arrays for better handling
    targetPose = np.array(targetPoseList)
    currentTarget = np.array(currentTargetList)
    currentPose = np.array(currentPoseList)

    # plot time dependend
    plt.figure()

    t = targetPose[:, 0]
    x = targetPose[:, 1]
    plt.plot(t, x, "r", label="theoretical target")

    t = currentTarget[:, 0]
    x = currentTarget[:, 1]
    plt.plot(t, x, "k-.", label="target")

    t = currentPose[:, 0]
    x = currentPose[:, 1]
    plt.plot(t, x, "b", label="measured")

    plt.grid()
    plt.xlabel('t in s')
    plt.ylabel('x position in m')
    plt.legend(loc="upper right")
    plt.show()

if __name__ == '__main__':
    try:

        # default values
        usePoseController = False
        trajectory = "circle"
        tPose = -1

        # parse arguments
        i = 0
        while i < len(sys.argv):
            arg = sys.argv[i]
            if i == 0:
                pass	# program name is no for us relevant parameter

            elif arg == "-i" or arg == "-I":
                pass	# nothing to do, as usePoseController is alredy false and thus impedance control active
            elif arg == "-p" or arg == "-P":
                usePoseController = True
            else:
                print("Unknown argument: " + str(arg))
                exit()

            i += 1

        # inform user
        print("Starting step response recording on axis X")

        # execute
        main(usePoseController)

    except rospy.ROSInterruptException:
        pass
