#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 20.12.2022

Ros (Robot) Interface for Exudyn
"""

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped

import sys
import os

from util.scripts.util.common import createPoseStampedMsg
from util.msg import segmentCommand

class RosInterface:

    ## constructor
    def __init__(self, useImpedanceController):

        # initialize some variables

        # force calibration
        self.effortsCalibrated = False
        self.nDesiredCalibrationValues = 100    # external efforts are sent with 30Hz from the robot, so calibration progress takes nCalibrationValues/30 seconds to finish
        self.calibrationValues = np.zeros(shape=(self.nDesiredCalibrationValues, 6))
        self.nGotCalibrationValues = 0   # iteration variable

        # global variable for external forces and moments (combined => efforts)
        self.extEfforts = np.zeros(shape=(6, 1))

        # measured force and torque by robot are not equal zero at rest, so store first measured force and torque (assumption: at rest) and substract them of every other one measured
        self.effortsOffset = np.zeros(shape=(6, 1))
        self.effortsThreshold = np.zeros(shape=(6, 1))
        self.effortsThresholdFactor = 2  # factor for scaling threshold

        # start position of robot in robot base frame in meters
        self.globalStartPos = np.array([0.4, 0, 0.2])    # default, but not precise enough
        self.globalStartPosSet = False

        self.posOffset = np.array([0, 0, 0]) # offset between user-interact position in simulation and robot world space 
        self.firstPose = True                # flag to determine first step; needed to calculate posOffset
        #self.T = np.eye(4)                   # for full coordinate transformation; currently not used

        self.impedanceController = useImpedanceController

        self.lastStepTime = 0    # last step time (exudyn time)

        self.logFile = None

        # init ros
        rospy.init_node('ExudynExample3_1', anonymous=True)

        if self.impedanceController:
            # publisher for pendulum poses (=endeffector positions)
            self.pub = rospy.Publisher('/my_cartesian_impedance_controller/setTargetPose', PoseStamped, queue_size=1000)
            self.pubF = rospy.Publisher('/my_cartesian_impedance_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)

            # subscriber for current pose
            self.globalStartPosSub = rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, self.currentPoseCallback)

            # open and init log file (csv)
            self.logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynIC.csv", "w")
        else:
            # publisher for pendulum poses (=endeffector positions)
            self.pub = rospy.Publisher('/my_cartesian_velocity_controller/setTargetPose', segmentCommand, queue_size=1000)
            self.pubF = rospy.Publisher('/my_cartesian_velocity_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)

            # subscriber for current pose
            self.globalStartPosSub = rospy.Subscriber("/my_cartesian_velocity_controller/getCurrentPose", PoseStamped, self.currentPoseCallback)

            # open and init log file (csv)
            self.logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynVC.csv", "w")

        # subscriber for external forces
        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.externalEffortCallback)
        
        self.logFile.write("rt,dt,px,py,pz,vx,vy,vz,ax,ay,az\n")

        # calibrate robot
        print("Calibrating. Do not touch robot")
        
        # wait for global start position
        print("Waiting for global start position")
        t0 = rospy.Time.now()

        while not self.globalStartPosSet:
            if rospy.Time.now() - t0 > rospy.Duration(5):
                print("ERROR: Did not get any robot position after 5 seconds of waiting")
                exit(-1)

        self.globalStartPosSub.unregister()  # we don't need current pose anymore

        # wait for calibration to finish
        print("Waiting for effort calibration")
        while not self.effortsCalibrated:
            pass

        print("Efforts calibrated")
        print("Everything ready to go, start using robot now")


    ## destructor
    def __del__(self):
        self.logFile.close()


    def publish(self, posExu, vel, acc, angleX, tExu):

        # in first iteration, offset between exudyn local position and robot global position has to be determined
        if self.firstPose:

            self.posOffset = self.globalStartPos - posExu
            self.posOffset = np.expand_dims(self.posOffset, axis=1)

            # full coordinate transformation
            # TODO: don't forget rotation
            #T = np.concatenate((trafoMat, posOffset), axis=1)
            #T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
            self.firstPose = False

        # local exudyn position has to be transformed to global robot position
        # initialize container
        pos = [0, 0, 0]

        # as for now no rotation is required, it is faster to compute coordinates without matrix multiplication
        pos[0] = posExu[0] + self.posOffset[0][0]
        pos[1] = posExu[1] + self.posOffset[1][0]
        pos[2] = posExu[2] + self.posOffset[2][0]

        # compose message and publish
        tRos = rospy.Time.now()
        segmentDuration = tExu - self.lastStepTime     # time from last position to this position
        #print(segment_duration.to_sec(), "\t", t - self.lastStepTime)
        self.lastStepTime = tExu

        if self.impedanceController:
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

        self.pub.publish(msg)

        # publish current external force
        msgF = WrenchStamped()
        msgF.header.stamp = tRos
        msgF.wrench.force.x = self.extEfforts[0]
        msgF.wrench.force.y = self.extEfforts[1]
        msgF.wrench.force.z = self.extEfforts[2]
        self.pubF.publish(msgF)

        self.logFile.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(tRos.to_sec(), segmentDuration, posExu[0], posExu[1], posExu[2], vel[0], vel[1], vel[2], acc[0], acc[1], acc[2]))


    ## callbacks
    def currentPoseCallback(self, data):
        """
        callback function to set global robot position at beginning of program
        -> globalStartPos
        """

        if not self.globalStartPosSet:
            self.globalStartPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
            self.globalStartPosSet = True
            # TODO Logger
            print("Global start pos set")


    def externalEffortCallback(self, data):
        """
        callback function for external forces and torques, applied by the user on the robot
        for simplicity reasons, forces and torques are denoted as efforts here

        every time new external efforts are registered by the robot(which happens continuously), 
        they are published and received by this callback function. Here they are written into 
        a global variable, in order to be processed by exudyn

        :param geometry_msgs.msg.WrenchStamped data: received message
        """

        # before operation, calibrate forces
        if not self.effortsCalibrated:
            if self.nGotCalibrationValues < self.nDesiredCalibrationValues:

                self.calibrationValues[self.nGotCalibrationValues] = np.array([data.wrench.force.x,
                                                                               data.wrench.force.y,
                                                                               data.wrench.force.z,
                                                                               data.wrench.torque.x,
                                                                               data.wrench.torque.y,
                                                                               data.wrench.torque.z])
                self.nGotCalibrationValues += 1
            else:
                for i in range(6):
                    
                    # use mean value as offset
                    self.effortsOffset[i] = np.mean(self.calibrationValues[:,i])

                    # use peak to peak value as threshold
                    self.effortsThreshold[i] = np.abs(np.max(self.calibrationValues[:,i]) - np.min(self.calibrationValues[:,i]))*self.effortsThresholdFactor

                self.effortsCalibrated = True

                #for i in range(6):
                #    print("effort offset: ", effortsOffset[i])
                #    print("effort threshold: ", effortsThreshold[i])

                # TODO Logger
                # print("Effort calibration done")

        # operating mode
        else:
            
            efforts = np.zeros(shape=(6,))

            # get forces and substract offset
            efforts[0] = data.wrench.force.x - self.effortsOffset[0]
            efforts[1] = data.wrench.force.y - self.effortsOffset[1]
            efforts[2] = data.wrench.force.z - self.effortsOffset[2]

            # get torques and substract offset
            efforts[3] = data.wrench.torque.x - self.effortsOffset[3]
            efforts[4] = data.wrench.torque.y - self.effortsOffset[4]
            efforts[5] = data.wrench.torque.z - self.effortsOffset[5]

            # additional to force offset, there is also some noise on force/torque measurement

            effortNames = ["fx", "fy", "fz", "mx", "my", "mz"]

            for i in range(6):
                if abs(efforts[i]) > self.effortsThreshold[i]:
                    self.extEfforts[i] = efforts[i]
                    #print("effort", effortNames[i], "=", extEfforts[i])
                else:
                    self.extEfforts[i] = 0
            # TODO effort trafo with rot matrix