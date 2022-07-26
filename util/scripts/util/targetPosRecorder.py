#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 25.07.2022

Script for recording and analizing /my_cartesian_impedance_controller/setDesiredPose
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt


targetPoseList = []


def targetPoseCallback(data):
    global targetPoseList
    duration = data.header.stamp
    t = duration.secs + duration.nsecs*1e-9
    targetPoseList.append([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])
    

# init ros
rospy.init_node('testing2', anonymous=True)

# subscriber for external forces
rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)

# record for amount of seconds
duration = 1.7   # seconds
print("Recording rostopic /my_cartesian_impedance_controller/setDesiredPose for {} seconds\n".format(duration))
rospy.sleep(duration)

# for better handling
targetPose = np.asarray(targetPoseList)

# provide data space
dataShape = (targetPose.shape[0]-1, targetPose.shape[1]+2)
data = np.zeros(shape=dataShape)

for i in range(dataShape[0]):
    # time
    data[i][0] = targetPose[i+1][0] - targetPose[i][0]

    # pos differences in m on x y z axes
    for j in range(1, 4):
        data[i][j] = targetPose[i+1][j] - targetPose[i][j]

    # abs pos differences in m
    data[i][4] = np.sqrt(data[i][1]**2 + data[i][2]**2 + data[i][3]**2)

    # velocity
    data[i][5] = data[i][4] / data[i][0]

# print results
print("dt         max: {:.4e} s   \tmin: {:.4e} s   \tmean: {:.4e} s   \tstd dev: {:.4e} s  ".format(np.amax(np.abs(data[:,0])), np.amin(np.abs(data[:,0])), np.mean(data[:,0]),np.std(data[:,0])))
print("dx         max: {:.4e} m   \tmin: {:.4e} m   \tmean: {:.4e} m   \tstd dev: {:.4e} m  ".format(np.amax(np.abs(data[:,1])), np.amin(np.abs(data[:,1])), np.mean(data[:,1]),np.std(data[:,1])))
print("dy         max: {:.4e} m   \tmin: {:.4e} m   \tmean: {:.4e} m   \tstd dev: {:.4e} m  ".format(np.amax(np.abs(data[:,2])), np.amin(np.abs(data[:,2])), np.mean(data[:,2]),np.std(data[:,2])))
print("dz         max: {:.4e} m   \tmin: {:.4e} m   \tmean: {:.4e} m   \tstd dev: {:.4e} m  ".format(np.amax(np.abs(data[:,3])), np.amin(np.abs(data[:,3])), np.mean(data[:,3]),np.std(data[:,3])))
print("abs        max: {:.4e} m   \tmin: {:.4e} m   \tmean: {:.4e} m   \tstd dev: {:.4e} m  ".format(np.amax(np.abs(data[:,4])), np.amin(np.abs(data[:,4])), np.mean(data[:,4]),np.std(data[:,4])))
print("abs vel    max: {:.4e} m/s \tmin: {:.4e} m/s \tmean: {:.4e} m/s \tstd dev: {:.4e} m/s".format(np.amax(np.abs(data[:,5])), np.amin(np.abs(data[:,5])), np.mean(data[:,5]),np.std(data[:,5])))

# plot results
plt.figure()

# ty plot
plt.subplot(1, 4, 1)
plt.plot(data[:,2])

# tz plot
plt.subplot(1, 4, 2)
plt.plot(data[:,3])

# yz plot
plt.subplot(1, 4, 3)
plt.plot(data[:,2], data[:,3])
plt.axis('equal')

# vt diagramm

plt.subplot(1, 4, 4)
plt.plot(data[:,5])

plt.show()


"""
# only saving script I used to calculate ds from ros example pose controller

size = 10000

deltaX = np.empty(shape=(size, ))
deltaZ = np.empty(shape=(size, ))

firstX = 0
firstZ = 0

newX = firstX
newZ = firstZ

t = 0

dt = 0.0012
radius = 0.3

for i in range(size):
        
        t += dt

        angle = np.pi / 4 * (1 - np.cos(np.pi / 5.0 * t))
        delta_x = radius * np.sin(angle)
        delta_z = radius * (np.cos(angle) - 1)

        oldX = newX
        oldZ = newZ

        newX = firstX - delta_x
        newZ = firstZ - delta_z

        deltaX[i] = oldX - newX
        deltaZ[i] = oldZ - newZ

print(np.amax(np.abs(deltaX)), np.amin(np.abs(deltaX)))
print(np.amax(np.abs(deltaZ)), np.amin(np.abs(deltaZ)))
"""