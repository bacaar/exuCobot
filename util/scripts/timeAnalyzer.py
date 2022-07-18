#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 15.06.2022

Script for analyzing time delay between sending of target positions to robot and robot actually being there
"""

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np


targetPos = None
currentPosList = []
scriptStartTime = None

def targetPoseCallback(data):
    global targetPos
    if targetPos is None:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        targetPos = (t, data.pose.position.x, data.pose.position.y, data.pose.position.z)


def currentPoseCallback(data):
    duration = data.header.stamp - scriptStartTime
    t = duration.secs + duration.nsecs*1e-9
    currentPosList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def main(useThresholdMode=True):

    if useThresholdMode:
        print("Using threshold mode")
    else:
        print("Using smallest distance mode")

    rospy.init_node('timeAnalyzer', anonymous=True)

    global scriptStartTime
    scriptStartTime = rospy.Time.now()

    rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)

    while not rospy.is_shutdown():
        global targetPos

        found = -1 # variable to store indices of found currentpos

        if useThresholdMode:
            threshold = 0.01

            if targetPos is not None:

                for i, currentpos in enumerate(currentPosList):
                    if abs(currentpos[1] - targetPos[1]) < threshold and abs(currentpos[2] - targetPos[2]) < threshold and abs(currentpos[3] - targetPos[3]) < threshold:
                        #print("found corresponding position")
                        #print("target Pos: x{} y{} z{}".format(targetPos[1], targetPos[2], targetPos[3]))
                        #print("current Pos: x{} y{} z{}".format(currentpos[1], currentpos[2], currentpos[3]))
                        print("dt = ", currentpos[0]-targetPos[0])
                        found = i
                    else:
                        if found != -1: # a position has been found but this one isn't near the target position anymore, so any other one following isn't either
                            break

        else:   # search smallest distance

            if targetPos is not None:

                distance = 1000000  # initialize with big distance (actually not important anymore, just to be sure)
                for i, currentpos in enumerate(currentPosList):
                    err = np.sqrt((currentpos[1] - targetPos[1])**2 + (currentpos[2] - targetPos[2])**2 + (currentpos[3] - targetPos[3])**2)
                    if i == 0:
                        distance = err
                    else:
                        if err < distance:
                            distance = err
                        else:
                            #print("found corresponding position")
                            #print("target Pos: x{} y{} z{}".format(targetPos[1], targetPos[2], targetPos[3]))
                            #print("current Pos: x{} y{} z{}".format(currentPosList[i-1][1], currentPosList[i-1][2], currentPosList[i-1][3]))
                            print("dt = ", currentPosList[i-1][0]-targetPos[0])
                            found = i-1
                            break

        # delete all previous current positions
        if found != -1:
            #print("Len before deleting:", len(currentPosList))
            #print("Last found index", found)
            for _ in range(found+1):
                currentPosList.pop(0)
            #print("Len after deleting:", len(currentPosList))
            targetPos = None


if __name__ == "__main__":
    try:
        import sys
        useThresholdeMode = True    #default value
        if len(sys.argv) == 2:
            if sys.argv[1] == "-t":
                useThresholdeMode = True
            elif sys.argv[1] == "-s":
                useThresholdeMode = False   # use smallest distance method

        main(useThresholdeMode)
    except rospy.ROSInterruptException:
        pass