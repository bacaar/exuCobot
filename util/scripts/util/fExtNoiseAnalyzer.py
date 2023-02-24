#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 08.10.2022

Script for analyzing noise in measured force and torque

how to: start controller (exudyn not needed) and then start this script
"""

import rospy
from geometry_msgs.msg import WrenchStamped

import matplotlib.pyplot as plt

import numpy as np

scriptStartTime = None
isRecording = False

duration = 5
f = 30

# provide storage space
inputList = np.zeros(shape=(int(duration * f), 7))
nData = 0
lostData = 0

## define callback functions
def externalForceCallback(data):
    duration = data.header.stamp - scriptStartTime
    t = duration.to_sec()

    global inputList
    global nData
    global lostData

    if nData < inputList.shape[0]:
        inputList[nData] = (np.array([t, 
                                    data.wrench.force.x,
                                    data.wrench.force.z,
                                    data.wrench.force.y,
                                    data.wrench.torque.x,
                                    data.wrench.torque.y,
                                    data.wrench.torque.z]))
        nData += 1
    else:
        lostData += 1

def main():

    global isRecording
    global scriptStartTime

    rospy.init_node("fExtNoiseAnalyzer")

    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalForceCallback)    # force published by franka controller

    print("Starting recording for {} seconds".format(duration))
    scriptStartTime = rospy.Time.now()
    isRecording = True
    rospy.sleep(duration)
    isRecording = False
    print("Finished recording")

    print("recorded data: ", nData)
    print("lost data: ", lostData)

    names = ["fx", "fy", "fz", "tx", "ty", "tz"]
    for i in range(1,7):
        data = inputList[:nData,i]
        print(names[i-1], "min:", np.min(data), 
                          "\tmax:", np.max(data), 
                          "\tpk-pk:", np.abs(np.max(data)-np.min(data)),
                          "\tmean:", np.max(data),
                          "\tstd:", np.std(data))

    fig, axs = plt.subplots(3,2,sharex=True)

    labels = ["x", "y", "z"]
    for i in range(3):
        axs[i][0].plot(inputList[:nData,0], inputList[:nData,i+1])
        axs[i][1].plot(inputList[:nData,0], inputList[:nData,i+4])
        axs[i][0].set_ylabel(labels[i])
        axs[i][0].grid()    
        axs[i][1].grid()


    axs[0][0].set_title("force in N")
    axs[0][1].set_title("torque in Nm")

    axs[2][0].set_xlabel("time in s")
    axs[2][1].set_xlabel("time in s")

    plt.show()


if __name__ == "__main__":
    main()