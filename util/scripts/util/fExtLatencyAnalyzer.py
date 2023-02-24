#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 25.07.2022

Script for analyzing where latency between force-input and motion comes from
"""

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from util.msg import segmentCommand

import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from mpl_axes_aligner import align

# provide storage space
forceInputList = []
forceInputList2 = []
targetPoseList = []
currentPoseList = []

scriptStartTime = None
lastTime = 0    # needed for velocity controller as target there has no time stamp
isRecording = False


## define callback functions
def externalForceCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        
        # get forces
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z

        forceInputList.append([t, fx, fy, fz])

def externalForceCallback2(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9

        # get forces
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z

        forceInputList2.append([t, fz, fx, fy])


def targetPoseCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        targetPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def targetPoseCallback2(data):
    if isRecording:
        global lastTime
        t = lastTime + data.dt
        lastTime = t
        targetPoseList.append([t, data.x.pos, data.y.pos, data.z.pos])


def currentPoseCallback(data):
    if isRecording:
        duration = data.header.stamp - scriptStartTime
        t = duration.secs + duration.nsecs*1e-9
        currentPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])


def main():
    rospy.init_node('fExtLatencyAnalyzer', anonymous=True)
    global scriptStartTime
    global isRecording
    scriptStartTime = rospy.Time.now()

    plotYOnly = True

    # duration of recording
    duration = 10

    # setup subscribers
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalForceCallback)    # force published by franka controller
    rospy.Subscriber("/my_cartesian_impedance_controller/analysis/getExternalForce", WrenchStamped, externalForceCallback2)       # force received by exudyn
    rospy.Subscriber("/my_cartesian_impedance_controller/setTargetPose", PoseStamped, targetPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)
    rospy.Subscriber("/my_cartesian_velocity_controller/analysis/getExternalForce", WrenchStamped, externalForceCallback2)       # force received by exudyn
    rospy.Subscriber("/my_cartesian_velocity_controller/setTargetPose", segmentCommand, targetPoseCallback2)
    rospy.Subscriber("/my_cartesian_velocity_controller/getCurrentPose", PoseStamped, currentPoseCallback)

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

    forceInput = np.array(forceInputList)
    forceInput2 = np.array(forceInputList2)
    targetPose = np.array(targetPoseList)
    currentPose = np.array(currentPoseList)

    # assuming that the robot is not moving at the beginning of the recording, set first position == zero
    targetPose[:,2] -= targetPose[0,2]
    currentPose[:,2] -= currentPose[0,2]

    # let all time vectors start at zero
    forceInput[:,0] -= forceInput[0,0]
    forceInput2[:,0] -= forceInput2[0,0]
    targetPose[:,0] -= targetPose[0,0]
    currentPose[:,0] -= currentPose[0,0]

    df_f1 = pd.DataFrame(forceInput, columns=["t","x","y","z"])
    df_f2 = pd.DataFrame(forceInput2, columns=["t","x","y","z"])
    df_tp = pd.DataFrame(targetPose, columns=["t","x","y","z"])
    df_cp = pd.DataFrame(currentPose, columns=["t","x","y","z"])

    df_f1.to_csv("f1.csv")
    df_f2.to_csv("f2.csv")
    df_tp.to_csv("tp.csv")
    df_cp.to_csv("cp.csv")

    # plot trajectories
    if plotYOnly:
        fig, ax = plt.subplots()
    else:
        fig, ax = plt.subplots(1, 3)

    if plotYOnly:
        ax.plot(targetPose[:, 0], targetPose[:, 2])
        ax.plot(currentPose[:, 0], currentPose[:, 2])

        #ax.legend(["target pos", "current pos"])

        ax.set_title("y-axis")
        ax.set_xlabel("t in s")
        ax.set_ylabel("pos in m")

        ax.grid()

        # plot force input on secondary axis
        ax2 = ax.twinx()
        ax2.plot(forceInput[:, 0], forceInput[:, 2], "k", linewidth=0.5)

        ax2.plot(forceInput2[:, 0], forceInput2[:, 2], "r")
        ax2.set_ylabel("force input in N")

        #ax2.legend(["published", "received"])

        fig.legend(["target pos", "current pos", "published force", "received force"])

        # Adjust the plotting range of two y axes
        org1 = 0.0  # Origin of first axis
        org2 = 0.0  # Origin of second axis
        pos = 0.5  # Position the two origins are aligned
        align.yaxes(ax, org1, ax2, org2, pos)

    else:
        # plot x, y and z axes in different subplots
        for i_, axis in enumerate(["x", "y", "z"]):
            i = i_ + 1

            ax[i_].plot(targetPose[:, 0], targetPose[:, i])
            ax[i_].plot(currentPose[:, 0], currentPose[:, i])

            ax[i_].legend(["target pos", "current pos"])

            ax[i_].set_title(axis + "-axis")
            ax[i_].set_xlabel("t in s")
            ax[i_].set_ylabel("pos in m")

            ax[i_].grid()

            # plot force input on secondary axis
            ax2 = ax[i_].twinx()
            ax2.plot(forceInput[:, 0], forceInput[:, 2], "k")
            ax2.plot(forceInput2[:, 0], forceInput2[:, 2], "r")
            ax2.set_ylabel("force input in N")

    #plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()