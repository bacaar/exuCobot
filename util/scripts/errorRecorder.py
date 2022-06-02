#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 02.06.2022

Script for recording errors of robot following trajectory
"""

import rospy
from geometry_msgs.msg import Pose

import time

error = []
targetPose = []
currentPose = []

def targetPoseCallback(data):
    targetPose.append((data.position.x, data.position.y, data.position.z))


def currentPoseCallback(data):
    currentPose.append((data.position.x, data.position.y, data.position.z))


def currentErrorCallback(data):
    error.append((data.position.x, data.position.y, data.position.z))


def main():

    duration = 6    # duration of planned recording in seconds

    rospy.init_node('errorRecorder', anonymous=True)

    # setup subscirbers
    rospy.Subscriber("/my_cartesian_impedance_example_controller/setDesiredPose", Pose, targetPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_example_controller/getCurrentPose", Pose, currentPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_example_controller/getCurrentError", Pose, currentErrorCallback)

    t0 = time.time()

    # record data from topics for duration of time
    print("Starting recording")
    while(time.time() - t0 < duration):
        pass
    print("Finished recording")
    print("Target pose length: {}\t\tCurrent pose length: {}\t\tError length: {}".format(len(targetPose),
                                                                                         len(currentPose),
                                                                                         len(error)))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass