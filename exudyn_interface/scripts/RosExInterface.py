#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 20.12.2022

Interface between Exudyn and ROS (Robot)
"""

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped

import sys
import os

from util.scripts.util.common import createPoseStampedMsg
from util.msg import segmentCommand

# force calibration
effortsCalibrated = False
nDesiredCalibrationValues = 100    # external efforts are sent with 30Hz from the robot, so calibration progress takes nCalibrationValues/30 seconds to finish
calibrationValues = np.zeros(shape=(nDesiredCalibrationValues, 6))
nGotCalibrationValues = 0   # iteration variable

# global variable for external forces and moments (combined => efforts)
extEfforts = np.zeros(shape=(6, 1))

# measured force and torque by robot are not equal zero at rest, so store first measured force and torque (assumption: at rest) and substract them of every other one measured
effortsOffset = np.zeros(shape=(6, 1))
effortsThreshold = np.zeros(shape=(6, 1))
effortsThresholdFactor = 10  # factor for scaling threshold

# start position of robot in robot base frame in meters
globalStartPos = np.array([0.4, 0, 0.2])    # default, but not precise enough
globalStartPosSet = False
globalStartPosSub = None

impedanceController = False  # default

pub = None
pubF = None

lastStepTime = 0    # last step time (exudyn time)

logFile = None

def currentPoseCallback(data):
    """
    callback function to set global robot position at beginning of program
    -> globalStartPos
    """

    global globalStartPosSet
    global globalStartPos
    if not globalStartPosSet:
        globalStartPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        globalStartPosSet = True
        # TODO Logger
        print("Global start pos set to ", globalStartPos)


def externalEffortCallback(data):
    """
    callback function for external forces and torques, applied by the user on the robot
    for simplicity reasons, forces and torques are denoted as efforts here

    every time new external efforts are registered by the robot(which happens continuously), 
    they are published and received by this callback function. Here they are written into 
    a global variable, in order to be processed by exudyn

    :param geometry_msgs.msg.WrenchStamped data: received message
    """

    global effortsCalibrated
    global effortsOffset
    global effortsThreshold
    global nGotCalibrationValues

    # before operation, calibrate forces
    if not effortsCalibrated:
        if nGotCalibrationValues < nDesiredCalibrationValues:

            calibrationValues[nGotCalibrationValues] = np.array([data.wrench.force.x,
                                                                 data.wrench.force.y,
                                                                 data.wrench.force.z,
                                                                 data.wrench.torque.x,
                                                                 data.wrench.torque.y,
                                                                 data.wrench.torque.z])
            nGotCalibrationValues += 1
        else:
            for i in range(6):
                
                # use mean value as offset
                effortsOffset[i] = np.mean(calibrationValues[:,i])

                # use peak to peak value as threshold
                effortsThreshold[i] = np.abs(np.max(calibrationValues[:,i]) - np.min(calibrationValues[:,i]))*2#*effortsThresholdFactor

            effortsCalibrated = True

            #for i in range(6):
            #    print("effort offset: ", effortsOffset[i])
            #    print("effort threshold: ", effortsThreshold[i])

            # TODO Logger
            # print("Effort calibration done")

    # operating mode
    else:
        
        efforts = np.zeros(shape=(6,))

        # get forces and substract offset
        efforts[0] = data.wrench.force.x - effortsOffset[0]
        efforts[1] = data.wrench.force.y - effortsOffset[1]
        efforts[2] = data.wrench.force.z - effortsOffset[2]

        # get torques and substract offset
        efforts[3] = data.wrench.torque.x - effortsOffset[3]
        efforts[4] = data.wrench.torque.y - effortsOffset[4]
        efforts[5] = data.wrench.torque.z - effortsOffset[5]

        # additional to force offset, there is also some noise on force/torque measurement

        effortNames = ["fx", "fy", "fz", "mx", "my", "mz"]

        for i in range(6):
            if abs(efforts[i]) > effortsThreshold[i]:
                extEfforts[i] = efforts[i]
                #print("effort", effortNames[i], "=", extEfforts[i])
            else:
                extEfforts[i] = 0
        # TODO effort trafo with rot matrix

def getGlobalStartPos():
    return globalStartPos

def RosExInit():

    global pub
    global pubF
    global globalStartPosSub

    # init ros
    rospy.init_node('ExudynExample3_1', anonymous=True)

    # publisher for pendulum poses (=endeffector positions)
    if impedanceController:
        pub = rospy.Publisher('/my_cartesian_impedance_controller/setTargetPose', PoseStamped, queue_size=1000)
        pubF = rospy.Publisher('/my_cartesian_impedance_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)
    else:
        pub = rospy.Publisher('/my_cartesian_velocity_controller/setTargetPose', segmentCommand, queue_size=1000)
        pubF = rospy.Publisher('/my_cartesian_velocity_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)

    # subscriber for external forces
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalEffortCallback)

    # subscriber for current pose
    if impedanceController:
        globalStartPosSub = rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)
    else:
        globalStartPosSub = rospy.Subscriber("/my_cartesian_velocity_controller/getCurrentPose", PoseStamped, currentPoseCallback)

    # open and initialize log file (csv)
    global logFile
    if impedanceController:
        logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynIC.csv", "w")
    else:
        logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynVC.csv", "w")
    logFile.write("rt,dt,px,py,pz,vx,vy,vz,ax,ay,az\n")


def calibrate():
    # wait for global start position
    print("Waiting for global start position")
    t0 = rospy.Time.now()

    while not globalStartPosSet:
        if rospy.Time.now() - t0 > rospy.Duration(2):
            print("ERROR: Did not get any robot position after 2 seconds of waiting")
            return False

    globalStartPosSub.unregister()  # we don't need current pose anymore

    # wait for calibration to finish
    print("Waiting for effort calibration")
    while not effortsCalibrated:
        pass

    print("Everything ready to go, start using robot now")


def publish(pos, vel, acc, angleX, tExu):

    global lastStepTime

    # compose message and publish
    tRos = rospy.Time.now()
    segmentDuration = tExu - lastStepTime     # time from last position to this position
    #print(segment_duration.to_sec(), "\t", t - lastStepTime)
    lastStepTime = tExu

    if impedanceController:
        msg = createPoseStampedMsg(pos, (angleX, 0, 0), tRos)
    else:
        msg = segmentCommand()
        msg.x.pos = pos[0]
        msg.y.pos = pos[1]
        msg.z.pos = pos[2]

        msg.x.vel = vel[0]
        msg.y.vel = vel[1]
        msg.z.vel = vel[2]

        msg.x.acc = acc[0]
        msg.y.acc = acc[1]
        msg.z.acc = acc[2]

        msg.dt = segmentDuration

    pub.publish(msg)

    # publish current external force
    msgF = WrenchStamped()
    msgF.header.stamp = tRos
    msgF.wrench.force.x = extEfforts[0]
    msgF.wrench.force.y = extEfforts[1]
    msgF.wrench.force.z = extEfforts[2]
    pubF.publish(msgF)

    logFile.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(tRos.to_sec(), segmentDuration, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], acc[0], acc[1], acc[2]))


def cleanUp():
    global logFile
    logFile.close()