#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 06.09.2022

Script for observing position, velocity acceleration and jerk of robot in cartesian and space6
"""

import numpy as np

import matplotlib.pyplot as plt

import rospy
from util.msg import kinematicState3dStamped

recordingTime = 5   # s

# constants
v_max_cartesian_trans = 1.7 # m/s
v_max_cartesian_rot = 2.5   # rad/s
a_max_cartesian_trans = 13  # m/s²
a_max_cartesian_rot = 25   # rad/s²

f = 1000            # publish frequency for states
vectorSize = recordingTime * f
t = np.zeros(shape=(vectorSize, ))
states = np.zeros(shape=(vectorSize, 3, 4))

scriptStartTime = None
index = 0


def currentStateCallback(data):
    global index
    if index < vectorSize:
        duration = data.header.stamp - scriptStartTime
        t[index] = duration.to_sec()
        states[index][0] = np.array([data.state.x.pos, data.state.x.vel, data.state.x.acc, data.state.x.jerk])
        states[index][1] = np.array([data.state.y.pos, data.state.y.vel, data.state.y.acc, data.state.y.jerk])
        states[index][2] = np.array([data.state.z.pos, data.state.z.vel, data.state.z.acc, data.state.z.jerk])
        index += 1


def main():
    rospy.init_node("kinematicStateObserver")

    global scriptStartTime
    scriptStartTime = rospy.Time.now()

    rospy.Subscriber("/my_cartesian_velocity_controller/getCurrentState", kinematicState3dStamped, currentStateCallback)

    rospy.sleep(recordingTime)

    print(len(states))
    print(np.amax(t))

    fig, axs = plt.subplots(4, 3, sharex='col')
    for axis in [0, 1, 2]:  # x y z
        for derivative in [0, 1, 2, 3]:  # pos, vel, acc, jerk
            axs[derivative][axis].plot(t[:index], states[:index, axis, derivative])
    plt.show()


if __name__ == "__main__":
    main()