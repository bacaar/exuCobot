#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 15.06.2022

Script for analyzing time delay between sending of target positions to robot and robot actually being there
"""

import rospy
from geometry_msgs.msg import PoseStamped


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


def main():

    rospy.init_node('timeAnalyzer', anonymous=True)

    global scriptStartTime
    scriptStartTime = rospy.Time.now()

    rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)

    xIncreasing = True
    yIncreasing = True
    zIncreasing = True

    dx = 1000
    dy = 1000
    dz = 1000

    threshold = 0.01

    while not rospy.is_shutdown():
        global targetPos

        if targetPos is not None:

            found = -1 # variable to store indices of found currentpos

            for i, currentpos in enumerate(currentPosList):
                if abs(currentpos[1] - targetPos[1]) < threshold and abs(currentpos[2] - targetPos[2]) < threshold and abs(currentpos[3] - targetPos[3]) < threshold:
                    print("found corresponding position")
                    print("target Pos: x{} y{} z{}".format(targetPos[1], targetPos[2], targetPos[3]))
                    print("current Pos: x{} y{} z{}".format(currentpos[1], currentpos[2], currentpos[3]))
                    print("dt = ", currentpos[0]-targetPos[0])
                    found = i
                else:
                    if found != -1: # a position has been found but this one isn't near the target position anymore, so any other one following isn't either
                        break

            # delete all previous current positions
            if found != -1:
                print("Len before deleting:", len(currentPosList))
                print("Last found index", found)
                for _ in range(found+1):
                    currentPosList.pop(0)
                print("Len after deleting:", len(currentPosList))





if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass