#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 10.01.2023

Script for recording robot movement
"""

import sys

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

scriptStartTime = None	# variable for storing start time of script
isRecording = False		# flag to enable/disable recording in Callbacks

externalTargetList = []
controllerTargetList = []
controllerTrajectoryList = []
robotPoseList = []


## define callback functions

def externalTargetCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        externalTargetList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])

def controllerTargetCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        controllerTargetList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])

def controllerTrajectoryCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        controllerTrajectoryList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])


def robotPoseCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        robotPoseList.append([duration.to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])


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
    rospy.Subscriber("/my_cartesian_impedance_controller/setTargetPose", PoseStamped, externalTargetCallback)			# target set by external
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentTarget", PoseStamped, controllerTargetCallback)	# target registered by controller
    rospy.Subscriber("/my_cartesian_impedance_controller/getEvaluatedTrajectory", PoseStamped, controllerTrajectoryCallback)	# evaluated trajectory by controller
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, robotPoseCallback)			# current position of robot controller

    # record data from topics for duration of time
    print("Starting recording for {} seconds".format(t))
    t0 = rospy.Time.now()
    isRecording = True
    rospy.sleep(t)
    isRecording = False
    print("Finished recording")
    print("External Target length: {}\t\tCurrent pose length: {}".format(len(externalTargetList),
                                                                         len(robotPoseList)))
    # convert lists to np.arrays for better handling
    externalTarget = np.array(externalTargetList)
    robotPose = np.array(robotPoseList)

    # extract data, needed twice afterwards
    target_t = externalTarget[:,0]
    target_x = externalTarget[:,1]
    target_y = externalTarget[:,2]
    target_z = externalTarget[:,3]

    robot_t = robotPose[:,0]
    robot_x = robotPose[:,1]
    robot_y = robotPose[:,2]
    robot_z = robotPose[:,3]

    # store data
    df_target = pd.DataFrame()
    df_robot = pd.DataFrame()

    df_target["target_t"] = target_t.tolist()
    df_target["target_x"] = target_x.tolist()
    df_target["target_y"] = target_y.tolist()
    df_target["target_z"] = target_z.tolist()

    df_robot["robot_t"] = robot_t.tolist()
    df_robot["robot_x"] = robot_x.tolist()
    df_robot["robot_y"] = robot_y.tolist()
    df_robot["robot_z"] = robot_z.tolist()

    df_target.to_csv("target.csv")
    df_robot.to_csv("robot.csv")

    # plot trajectories
    fig, ax = plt.subplots(1, 2)

    # plot xt
    ax[0].plot(target_t, target_x, "-", label="external target x")
    #ax.plot(target_t, target_y, "bx", label="sent by external")
    #ax.plot(target_t, target_z, "gx", label="exu target z")

    ax[0].plot(robot_t, robot_x, "-", label="measured x")

    ax[0].grid()
    ax[0].set_xlabel('t in s')
    ax[0].set_ylabel('position x in m')

    ax[0].legend(loc="upper left")

    # plot xy
    ax[1].plot(target_x, target_y, "-", label="external target x")
    ax[1].plot(robot_x, robot_y, "-", label="measured x")

    ax[1].grid()
    ax[1].set_xlabel('position x in m')
    ax[1].set_ylabel('position y in m')
    ax[1].legend(loc="upper left")

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
