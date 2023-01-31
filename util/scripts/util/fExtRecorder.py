#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 30.01.2023

Script for analyzing static force offset in measured force
"""

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from util.msg import segmentCommand

import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from mpl_axes_aligner import align

# provide storage space
effortList = []
jointPosList = []
poseList = []

scriptStartTime = None
lastTime = 0    # needed for velocity controller as target there has no time stamp
isRecording = False


## define callback functions
def externalEffortCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        
        effortList.append([t, data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])


def jointStateCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        jointPosList.append([t, data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5], data.position[6]])


def cartesianPoseCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        poseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def main():
    rospy.init_node('fExtRecorder', anonymous=True)
    global scriptStartTime
    global isRecording
    scriptStartTime = rospy.Time.now()

    plotYOnly = True

    # duration of recording
    duration = 3

    # setup subscribers
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalEffortCallback)    # force published by franka controller
    rospy.Subscriber("/franka_state_controller/joint_states", JointState, jointStateCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, cartesianPoseCallback)
    

    # record data from topics for duration of time
    print("Starting recording for {} seconds".format(duration))
    isRecording = True
    rospy.sleep(duration)
    isRecording = False
    print("Finished recording")

    # as only forces are registered by simulation, which abs is higher than 5N,
    # set all values of force array to 0 if smaller
    #threshold = 3
    #for i in range(len(forceInputList)):
    #    for j in range(1, 4):
    #        if np.abs(forceInputList[i][j]) < threshold:
    #            forceInputList[i][j] = 0.0

    efforts = np.array(effortList)
    joints = np.array(jointPosList)
    poses = np.array(poseList)

    # let time start at zero
    efforts[:,0] -= efforts[0,0]
    joints[:,0] -= joints[0,0]
    poses[:,0] -= poses[0,0]

    df_e = pd.DataFrame(efforts, columns=["t","fx","fy","fz","tx","ty","tz"])
    df_j = pd.DataFrame(joints, columns=["t","j0","j1","j2","j3","j4","j5","j6"])
    df_p = pd.DataFrame(poses, columns=["t","x","y","z"])

    df_e.to_csv("e.csv")
    df_j.to_csv("j.csv")
    df_p.to_csv("p.csv")

    mean_x = np.mean(efforts[:,1])
    mean_y = np.mean(efforts[:,2])
    mean_z = np.mean(efforts[:,3])

    print("mean x:", mean_x)
    print("mean y:", mean_y)
    print("mean z:", mean_z)

    fig, ax = plt.subplots(1,1)

    # plot x, y and z axes in different subplots
    for i_, axis in enumerate(["x", "y", "z"]):
        i = i_ + 1

        ax.plot(efforts[:, 0], efforts[:, i])
        #ax[i_].plot(currentPose[:, 0], currentPose[:, i])

        ax.legend(["x", "y", "z"])

        ax.set_title(axis + "-axis")
        ax.set_xlabel("t in s")
        ax.set_ylabel("pos in m")

        ax.grid()

        # plot force input on secondary axis
        #ax2 = ax[i_].twinx()
        #ax2.plot(forceInput[:, 0], forceInput[:, 2], "k")
        #ax2.plot(forceInput2[:, 0], forceInput2[:, 2], "r")
        #ax2.set_ylabel("force input in N")

    #plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()