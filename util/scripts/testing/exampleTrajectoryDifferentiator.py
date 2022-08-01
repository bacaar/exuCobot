#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 27.07.2022

Script for analyzing trajectory created by the ROS pose example controller

Original code:

double radius = 0.3;
double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
double delta_x = radius * std::sin(angle);
double delta_z = radius * (std::cos(angle) - 1);
std::array<double, 16> new_pose = initial_pose_;
new_pose[12] -= delta_x;
new_pose[14] -= delta_z;
"""

import matplotlib.pyplot as plt
import numpy as np

from geometry_msgs.msg import WrenchStamped


# initialize variables
firstX = 0
firstZ = 0

newX = firstX
newZ = firstZ

# parameter
dt = 0.001
radius = 0.3

size = 12000    # amount of datapoints

# provide some storage space
t = np.empty(shape=(size, ))

deltaX = np.zeros(shape=(size, ))
deltaZ = np.zeros(shape=(size, ))

posX = np.empty(shape=(size, ))
posZ = np.empty(shape=(size, ))

velX = np.empty(shape=(size, ))
velZ = np.empty(shape=(size, ))

accX = np.empty(shape=(size, ))
accZ = np.empty(shape=(size, ))

# analysis
for i in range(size):
        
        t[i] = dt * i

        angle = np.pi / 4 * (1 - np.cos(np.pi / 5.0 * t[i]))

        # position increment / decrement from starting position
        delta_x = radius * np.sin(angle)
        delta_z = radius * (np.cos(angle) - 1)

        # position
        posX[i] = firstX - delta_x
        posZ[i] = firstZ - delta_z

        if i > 0:
                # numerical differentiation to calulate velocity
                velX[i] = (posX[i] - posX[i-1])/dt
                velZ[i] = (posZ[i] - posZ[i-1])/dt


                deltaX[i] = posX[i] - posX[i-1]
                deltaZ[i] = posZ[i] - posZ[i-1]

        if i > 1:
                # numerical differentiation to calulate acceleration
                accX[i] = (velX[i] - velX[i-1])/dt
                accZ[i] = (velZ[i] - velZ[i-1])/dt

print("max step size x direction: ", np.amax(np.abs(deltaX)))
print("max step size z direction: ", np.amax(np.abs(deltaZ)))

# create plots
fig, axs = plt.subplots(4,1,sharex=True)

fig.suptitle("Pose example controller trajectory analysis")

# plot increment, decrement
axs[0].plot(t, deltaX, label='x')
axs[0].plot(t, deltaZ, label='z')
axs[0].set_ylabel('increment per step in m')
axs[0].grid()
axs[0].legend()

# plot absolute position
axs[1].plot(t, posX, label='x')
axs[1].plot(t, posZ, label='z')
axs[1].set_ylabel('position in m')
axs[1].grid()
axs[1].legend()

# plot velocity
axs[2].plot(t[1:], velX[1:], label='x')
axs[2].plot(t[1:], velZ[1:], label='z')
axs[2].set_ylabel('velocity in m/s')
axs[2].grid()
axs[2].legend()

# plot acceleration
axs[3].plot(t[2:], accX[2:], label='x')
axs[3].plot(t[2:], accZ[2:], label='z')
axs[3].set_ylabel('acceleration in m/s2')
axs[3].grid()
axs[3].legend()

axs[3].set_xlabel('time in s')


plt.show()