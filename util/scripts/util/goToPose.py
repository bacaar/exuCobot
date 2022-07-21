#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 19.07.2022

Script for moving robot into new pose by calling active controller
"""

import sys

import rospy
from geometry_msgs.msg import Pose, PoseStamped

import numpy as np
import tf

currentPose = Pose()
currentPoseSet = False  # flag for receiving pose first time

def createPoseStampedMsg(coords, euler, time):
    """
    function to create and return a PoseStamped-ros-msg

    :param coords: arbitrary container (tuple, list, np.array, ...) with x, y and z coordinates
    :param euler: arbitrary container (tuple, list, np.array, ...) with euler angles (pitch, roll and yaw)
    :param time: instance of ros class "Time" with time for message

    :return: the message object
    :rtype: geometry_msgs.msg.PoseStamped
    """

    # create message
    msg = PoseStamped()

    # write position into message
    msg.pose.position.x = coords[0]
    msg.pose.position.y = coords[1]
    msg.pose.position.z = coords[2]

    # endeffector should point straight down
    pitch = np.radians(euler[0])
    roll = np.radians(euler[1])
    yaw = np.radians(euler[2])

    # create Quaternion out of Euler angles
    quaternion = tf.transformations.quaternion_from_euler(pitch, yaw, roll)

    # only to be sure quaternion is correct
    assert np.linalg.norm(quaternion) == 1.0, "ERROR"

    # write orientation into message
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    # write current time into message
    msg.header.stamp = time

    return msg


def currentPoseCallback(data):

    global currentPose
    global currentPoseSet

    # save pose
    currentPose = data.pose

    currentPoseSet = True


def poseDif(pose1, pose2):
    dif = Pose()

    dif.position.x = pose1.position.x - pose2.position.x
    dif.position.y = pose1.position.y - pose2.position.y
    dif.position.z = pose1.position.z - pose2.position.z

    dif.orientation.x = pose1.orientation.x - pose2.orientation.x
    dif.orientation.y = pose1.orientation.y - pose2.orientation.y
    dif.orientation.z = pose1.orientation.z - pose2.orientation.z
    dif.orientation.w = pose1.orientation.w - pose2.orientation.w

    return dif

def absPosErr(pose1, pose2):

    dif = poseDif(pose1, pose2)

    absErr = np.sqrt(dif.position.x**2 + dif.position.y**2 + dif.position.z**2)

    return absErr, dif


def goToPose(x, y, z, pitchDeg, rollDeg, yawDeg, debug=False):

    # when called by another node, no new can be initialized
    # TODO: make this function a ROS service?
    try:
        rospy.init_node('goToPose', anonymous=True)
        rospy.Rate(1000)
    except rospy.exceptions.ROSException:
        print("Ros node alredy running, using existing one")

    # subscriber for current pose
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)

    # do not proceed until current pose has arrived
    print("Waiting for current pose")
    while not rospy.is_shutdown():
        if currentPoseSet:
            break

    # debug info
    if debug:
        print("Being at")
        print(currentPose)

    t0 = rospy.Time.now()
    msg = createPoseStampedMsg([x, y, z], [pitchDeg, rollDeg, yawDeg], t0)

    if debug:
        print("\nGoing to")
        print(msg.pose)

    finalTargetPose = msg.pose

    # initialize publisher for setting desired poses
    pub = rospy.Publisher('/my_cartesian_impedance_controller/setDesiredPose', PoseStamped, queue_size=10)

    totalDif = poseDif(finalTargetPose, currentPose)
    if debug:
        print("\nDifference")
        print(totalDif)

    # calculate in which directions axes have to move
    xDir = np.sign(totalDif.position.x)
    yDir = np.sign(totalDif.position.y)
    zDir = np.sign(totalDif.position.z)

    stepSize = 0.001

    i = 0
    while not rospy.is_shutdown():

        # update pose (target pose will be calculated by increasin / decreasing current pose)
        targetPose = currentPose

        # impedance control doesn't like it if difference between starting and end position is too big (limit not known)
        # if so, robot sometimes does unexpected and fast movements in arbitrary directions
        # therefor position has to be increased/decreased slowly
        # this problem does not occur on orientation, thus it has not to be handled

        if xDir == 1:   # if robot has to move in positive x direction
            if targetPose.position.x + stepSize <= finalTargetPose.position.x:
                targetPose.position.x += stepSize
            else:
                targetPose.position.x = finalTargetPose.position.x
        else:           # if robot has to move in negative x direction
            if targetPose.position.x - stepSize >= finalTargetPose.position.x:
                targetPose.position.x -= stepSize
            else:
                targetPose.position.x = finalTargetPose.position.x

        # same principle for y and z axes
        if yDir == 1:
            if targetPose.position.y + stepSize <= finalTargetPose.position.y:
                targetPose.position.y += stepSize
            else:
                targetPose.position.y = finalTargetPose.position.y
        else:
            if targetPose.position.y - stepSize >= finalTargetPose.position.y:
                targetPose.position.y -= stepSize
            else:
                targetPose.position.y = finalTargetPose.position.y

        if zDir == 1:
            if targetPose.position.z + stepSize <= finalTargetPose.position.z:
                targetPose.position.z += stepSize
            else:
                targetPose.position.z = finalTargetPose.position.z
        else:
            if targetPose.position.z - stepSize >= finalTargetPose.position.z:
                targetPose.position.z -= stepSize
            else:
                targetPose.position.z = finalTargetPose.position.z

        msg = createPoseStampedMsg([targetPose.position.x, targetPose.position.y, targetPose.position.z], [pitchDeg, rollDeg, yawDeg], t0)
        pub.publish(msg)
        i += 1

        absErrThreshold = stepSize*2
        absErr, err = absPosErr(finalTargetPose, currentPose)
        if debug:
            print("current x={:2.3} y={:2.3} z={:2.3}\t".format(currentPose.position.x, currentPose.position.y, currentPose.position.z), end="")
            print("error   x={:2.3} y={:2.3} z={:2.3}\tabs: {:2.4} (threshold={:2.4})".format(err.position.x, err.position.y, err.position.z, absErr, absErrThreshold))
        if absErr <= absErrThreshold:
            break

    if debug:
        print("Needed steps: " + str(i))



if __name__ == "__main__":
    try:
        if len(sys.argv) == 7:

            goToPose(float(sys.argv[1]),
                     float(sys.argv[2]),
                     float(sys.argv[3]),
                     float(sys.argv[4]),
                     float(sys.argv[5]),
                     float(sys.argv[6]),
                     True)

        else:
            print("Invalid arguments.")
            print("Usage: python3 goToPose.py [x in m] [y in m] [z in m] [pitch in deg] [roll in deg] [yaw in deg]")

    except rospy.ROSInterruptException:
        pass