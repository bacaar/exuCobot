#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 03.08.2022

Script for testing creation of polynomial trajectories
"""

import numpy as np
import sympy as sp

import rospy
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt

import time


scriptStartTime = None
sectionLength = rospy.Duration(0.01)
tSection = rospy.Duration(0.0)

# 1 dimensional vectors, look at y axis only
nextTwoPositions = np.zeros(shape=(2, ))
currentPos = 0
currentVel = 0
currentAcc = 0

nSamples = 5000    # number of samples to record; stop program after that
currentSampleNumber = 0
recordedPositions = np.zeros(shape=(nSamples, 2))

recordedTargets = np.zeros(shape=(nSamples, 2))
nRecordedTargets = 0

coefs = np.zeros(shape=(6, ))

firstIteration = True
gotFirstPosition = False


def calcCoefs(s0, ds0, dds0, sT, dsT, ddsT, T):

    T2 = T*T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

    mInv = np.array([[-6/T5, 6/T5, -3/T4, -3/T4, -1/(2*T3), 1/(2*T3)],
                        [15/T4, -15/T4, 8/T3, 7/T3, 3/(2*T2), -1/T2],
                        [-10/T3, 10/T3, -6/T2, -4/T2, -3/(2*T), 1/(2*T)],
                        [0, 0, 0, 0, 0.5, 0],
                        [0, 0, 1, 0, 0, 0],
                        [1, 0, 0, 0, 0, 0]])

    vec = np.array([s0, sT, ds0, dsT, dds0, ddsT])

    solution = mInv @ vec
    return solution


def evaluatePolynom(coef, t):

    A, B, C, D, E, F = coef

    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t

    s = A*t5 + B*t4 + C*t3 + D*t2 + E*t + F
    v = 5*A*t4 + 4*B*t3 + 3*C*t2 + 2*D*t + E
    a = 20*A*t3 + 12*B*t2 + 6*C*t + 2*D

    return np.array([s, v, a])


def targetPoseCallback(data):
    # TODO: currently time isn't registered: for more precise calculation, take time difference to current pos into account? does this make sense?

    global gotFirstPosition
    gotFirstPosition = True

    global nextTwoPositions    
    nextTwoPositions[0] = nextTwoPositions[1]
    #nextTwoPositions[1] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    nextTwoPositions[1] = data.pose.position.y

    global firstIteration
    global currentPos
    if firstIteration:
        currentPos = data.pose.position.y
        firstIteration = False

    nextVelocity = (nextTwoPositions[1]-currentPos)/sectionLength.to_sec()
    #nextVelocity = 0

    global coefs
    coefs = calcCoefs(currentPos, currentVel, currentAcc, nextTwoPositions[0], nextVelocity, 0, sectionLength.to_sec())

    global tSection
    tSection = rospy.Duration(0.0)

    global recordedTargets
    global nRecordedTargets

    if nRecordedTargets >= nSamples:
        exit()

    #recordedTargets[nRecordedTargets] = np.array([(data.header.stamp - scriptStartTime).to_sec(), data.pose.position.x, data.pose.position.y, data.pose.position.z])
    recordedTargets[nRecordedTargets] = np.array([(data.header.stamp - scriptStartTime).to_sec(), data.pose.position.y])
    nRecordedTargets += 1


def currentPoseCallback(data):
    global currentPos
    #currentPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    currentPos = data.pose.position.y

    duration = data.header.stamp - scriptStartTime
    t = duration.to_secs()
    global currentSampleNumber
    if currentSampleNumber >= nSamples:
        exit()
    recordedPositions[currentSampleNumber] = np.array([t, data.pose.position.x, data.pose.position.y, data.pose.position.z])
    recordedPositions[currentSampleNumber] = data.pose.position.y


def main():
    rospy.init_node('PolynomialTrajectoryTest', anonymous=True)

    # provide some space for coordinates (needed for plotting)
    pos = np.zeros(shape=(nSamples, ))
    vel = np.zeros(shape=(nSamples, ))
    acc = np.zeros(shape=(nSamples, ))
    
    tVec = np.zeros(shape=(nSamples, ))

    global coefs
    global tSection

    #tMeasured = np.zeros(shape=(nSamples, ))

    j = 0

    global scriptStartTime
    scriptStartTime = rospy.Time.now()
    t = rospy.Duration(0.0)     # start time

    global currentPos
    global currentVel
    global currentAcc

    # setup subscirbers
    rospy.Subscriber("/my_cartesian_impedance_controller/setDesiredPose", PoseStamped, targetPoseCallback)
    rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)

    # loop which gets executed once every 1ms
    while True:

        t0 = time.time()
        
        currentPos, currentVel, currentAcc = evaluatePolynom(coef=coefs, t=tSection.to_sec())

        dt = rospy.Time.now() - t - scriptStartTime
        t += dt
        tSection += dt

        if j < nSamples and gotFirstPosition:    
            pos[j] = currentPos
            vel[j] = currentVel
            acc[j] = currentAcc

            tVec[j] = t.to_sec()
        
        if j >= nSamples:
            break

        j += 1

        # assurance for 1ms loop
        while time.time() - t0 < 0.001:
            pass

    #print("tMeasured max: {}s, avg: {}s, std:{}s".format(np.amax(tMeasured), np.mean(tMeasured), np.std(tMeasured)))

    np.savetxt("exData", recordedTargets[:nRecordedTargets,:])

    fig, axs = plt.subplots(3, 1, sharex=True, sharey='row')
    labels = ["s in m", "v in m/s", "a in m/s2"]
    axs[0].plot(tVec[1:j], pos[1:j])
    #axs[0].plot(recordedTargets[:,0], recordedTargets[0:,1], 'kx')
    axs[0].plot(recordedTargets[:nRecordedTargets,0], recordedTargets[:nRecordedTargets,1], 'kx')
    #axs[0].plot(recordedTargets[:,0], recordedTargets[:,3], 'kx')
    axs[0].set_ylim(-0.5, 0.5)
    axs[0].set_ylabel(labels[0])
    axs[0].grid()
    axs[1].plot(tVec, vel)
    axs[1].set_ylabel(labels[1])
    axs[1].grid()
    axs[2].plot(tVec, acc)
    axs[2].set_ylabel(labels[2])
    axs[2].grid()
    axs[2].set_xlabel("t in s")
    #axs[0].set_title("end velocity 1")
        
    fig.set_tight_layout(True)
    plt.show()


if __name__ == "__main__":
    main()