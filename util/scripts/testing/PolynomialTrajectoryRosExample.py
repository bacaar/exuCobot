#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 03.08.2022

Script for testing creation of polynomial trajectories according to PR Angewandte Robotik (WS2019)
"""

import numpy as np
import sympy as sp

import rospy

import matplotlib.pyplot as plt


def calcCoefs(s0, ds0, dds0, sT, dsT, ddsT, T):

    T2 = T*T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

    m = np.array([[0, 0, 0, 0, 0, 1],
                  [T5, T4, T3, T2, T, 1],
                  [0, 0, 0, 0, 1, 0],
                  [5*T4, 4*T3, 3*T2, 2*T, 1, 0],
                  [0, 0, 0, 2, 0, 0],
                  [20*T3, 12*T2, 6*T, 2, 0, 0]])

    vec = np.array([s0, sT, ds0, dsT, dds0, ddsT])

    solution = np.linalg.inv(m) @ vec
    return solution


rosPos = np.array([0.15,
                   #0.15,
                   0.16,
                   0.17,
                   0.18,
                   #0.19,
                   0.20,
                   0.20,
                   0.20])



nextTwoPositions = np.zeros(shape=(2, ))

# poynomial coefficients
coefs = np.zeros(shape=(6, ))
dCoefs = np.array([5, 4, 3, 2, 1, 0])
ddCoefs = np.array([20, 12, 6, 2, 0, 0])


def evaluatePolynom(coef, t):

    A, B, C, D, E, F = coef

    s = 0
    v = 0
    a = 0

    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t


    #for i in range(6):
    #    s += coef[-(i+1)] * t**i
    #    if i > 1:
    #        v += dCoefs[-(i+1)] * coef[-(i+1)] * t**(i-1)

    #    if i > 2:
    #        a += ddCoefs[-(i+1)] * coef[-(i+1)] * t**(i-2)

    s = A*t5 + B*t4 + C*t3 + D*t2 + E*t + F
    v = 5*A*t4 + 4*B*t3 + 3*C*t2 + 2*D*t + E
    a = 20*A*t3 + 12*B*t2 + 6*C*t + 2*D

    return np.array([s, v, a])


def main():
    rospy.init_node('PolynomialTrajectoryTest', anonymous=True)

    currentPos = rosPos[0]
    currentVel =  0
    currentAcc =  0

    sectionLength = rospy.Duration(0.01)
    tSection = rospy.Duration(0.0)
    dt = rospy.Duration(0.00001)  # controller time steps
    stepsPerSection = int(round(sectionLength.to_sec() / dt.to_sec()))

    steps = int((len(rosPos) - 2) * sectionLength.to_sec() / dt.to_sec())

    # provide some space for coordinates (needed for plotting)
    pos = np.zeros(shape=(steps, ))
    vel = np.zeros(shape=(steps, ))
    acc = np.zeros(shape=(steps, ))
    tVec = np.zeros(shape=(steps, ))
    
    t = rospy.Duration(0.0)     # start time

    index = 1
    # loop which gets executed once every 1ms
    for j in range(steps):

        if j >= pos.shape[0]:
            print("Exceeded storage space at t=",t)
            break
        
        if j == 2000:
            h = 1
            h = 2

        if j == 0 or ((j-1) % stepsPerSection == 0 and j != 1):

            if j != 0:
                """
                fig, axs = plt.subplots(3, 1, sharex=True)
                labels = ["s in m", "v in m/s", "a in m/s2"]
                axs[0].plot(tVec[j-stepsPerSection:j], pos[j-stepsPerSection:j])
                axs[0].set_ylabel(labels[0])
                axs[0].grid()
                axs[1].plot(tVec[j-stepsPerSection:j], vel[j-stepsPerSection:j])
                axs[1].set_ylabel(labels[1])
                axs[1].grid()
                axs[2].plot(tVec[j-stepsPerSection:j], acc[j-stepsPerSection:j])
                axs[2].set_ylabel(labels[2])
                axs[2].grid()
                """

            nextTwoPositions[0] = rosPos[index]
            nextTwoPositions[1] = rosPos[index + 1]
            index += 1
            nextVelocity = (nextTwoPositions[1]-nextTwoPositions[0])/0.01

            global coefs
            coefs = calcCoefs(currentPos, currentVel, currentAcc, nextTwoPositions[0], nextVelocity, 0, sectionLength.to_sec())
            if j == 0:
                tSection = rospy.Duration(0.0)
            else:
                tSection = dt
        
        currentPos, currentVel, currentAcc = evaluatePolynom(coef=coefs, t=tSection.to_sec())
            
        pos[j] = currentPos
        vel[j] = currentVel
        acc[j] = currentAcc
        tVec[j] = t.to_sec()

        t += dt
        tSection += dt



    fig, axs = plt.subplots(3, 1, sharex=True)
    labels = ["s in m", "v in m/s", "a in m/s2"]
    axs[0].plot(tVec, pos)
    axs[0].set_ylabel(labels[0])
    axs[0].grid()
    axs[1].plot(tVec, vel)
    axs[1].set_ylabel(labels[1])
    axs[1].grid()
    axs[2].plot(tVec, acc)
    axs[2].set_ylabel(labels[2])
    axs[2].grid()
        
    plt.show()


if __name__ == "__main__":
    main()