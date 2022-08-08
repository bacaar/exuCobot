#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 03.08.2022

Script for testing creation of polynomial trajectories
"""

import numpy as np
import sympy as sp

import rospy

import matplotlib.pyplot as plt

import time


polyOrder = 5

def calcCoefs(s0, ds0, dds0, ddds0, sT, dsT, ddsT, dddsT, T):

    T2 = T*T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    T6 = T5 * T
    T7 = T6 * T

    if polyOrder == 3:
        m = np.array([[0, 0, 0, 1],
                      [T3, T2, T, 1],
                      [0, 2, 0, 0],
                      [6*T, 2, 0, 0]])
                    
        vec = np.array([s0, sT, dds0, ddsT])

        solution = np.linalg.inv(m) @ vec
        return solution

    if polyOrder == 5:

        mInv = np.array([[-6/T5, 6/T5, -3/T4, -3/T4, -1/(2*T3), 1/(2*T3)],
                         [15/T4, -15/T4, 8/T3, 7/T3, 3/(2*T2), -1/T2],
                         [-10/T3, 10/T3, -6/T2, -4/T2, -3/(2*T), 1/(2*T)],
                         [0, 0, 0, 0, 0.5, 0],
                         [0, 0, 1, 0, 0, 0],
                         [1, 0, 0, 0, 0, 0]])

        vec = np.array([s0, sT, ds0, dsT, dds0, ddsT])

        solution = mInv @ vec
        return solution

    if polyOrder == 7:

        m = np.array([[0, 0, 0, 0, 0, 0, 0, 1],
                      [T7, T6, T5, T4, T3, T2, T, 1],
                      [0, 0, 0, 0, 0, 0, 1, 0],
                      [7*T6, 6*T5, 5*T4, 4*T3, 3*T2, 2*T, 1, 0],
                      [0, 0, 0, 0, 0, 2, 0, 0],
                      [42*T5, 30*T4, 20*T3, 12*T2, 6*T, 2, 0, 0],
                      [0, 0, 0, 0, 6, 0, 0, 0],
                      [210*T4, 120*T3, 60*T2, 24*T, 6, 0, 0, 0]])

        vec = np.array([s0, sT, ds0, dsT, dds0, ddsT, ddds0, dddsT])

        solution = np.linalg.inv(m) @ vec
        return solution


def evaluatePolynom(coef, t):

    if polyOrder == 3:

        A, B, C, D = coef

        t2 = t * t
        t3 = t2 * t

        s = A*t3 + B*t2 + C*t + D
        v = 3*A*t2 + 2*B*t + C
        a = 6*A*t + 2*B

        return np.array([s, v, a])

    if polyOrder == 5:

        A, B, C, D, E, F = coef

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        s = A*t5 + B*t4 + C*t3 + D*t2 + E*t + F
        v = 5*A*t4 + 4*B*t3 + 3*C*t2 + 2*D*t + E
        a = 20*A*t3 + 12*B*t2 + 6*C*t + 2*D

        return np.array([s, v, a])

    if polyOrder == 7:

        A, B, C, D, E, F, G, H = coef

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        t6 = t5 * t
        t7 = t6 * t

        s = A*t7 + B*t6 + C*t5 + D*t4 + E*t3 + F*t2 + G*t + H
        v = 7*A*t6 + 6*B*t5 + 5*C*t4 + 4*D*t3 + 3*E*t2 + 2*F*t + G
        a = 2*A*t5 + 30*B*t4 + 20*C*t3 + 12*D*t2 + 6*E*t + 2*F

        return np.array([s, v, a])


def main():
    rospy.init_node('PolynomialTrajectoryTest', anonymous=True)

    simple = False

    if simple:
        targetPos = np.array([0.15,
                        0.15,
                        0.16,
                        0.17,
                        0.16,
                        0.18,
                        0.19,
                        0.19,
                        0.18])

    else:
        targetPos = np.array([3.103186422778971831e-01 ,
                            3.069528835455630489e-01 ,
                            3.031529369879830682e-01 ,
                            2.989225506480025496e-01 ,
                            2.942659709824386738e-01 ,
                            2.891879607586396173e-01 ,
                            2.836938175808958462e-01 ,
                            2.777893927103276539e-01 ,
                            2.714811098252267740e-01 ,
                            2.647759833561847964e-01 ,
                            2.576816360220383073e-01 ,
                            2.502063151889174497e-01 ,
                            2.423589076757228788e-01 ,
                            2.341489526353390804e-01 ,
                            2.255866521519502843e-01 ,
                            2.166828792109615209e-01 ,
                            2.074491827192652682e-01 ,
                            1.978977892798104987e-01 ,
                            1.880416014554331383e-01 ,
                            1.778941922924680563e-01 ,
                            1.674697959143922610e-01 ,
                            1.567832940392318575e-01 ,
                            1.458501983211817343e-01 ,
                            1.346866284662335822e-01 ,
                            1.233092861229128090e-01 ,
                            1.117354246017716601e-01 ,
                            9.998281453028290411e-02 ,
                            8.806970560240912071e-02 ,
                            7.601478463356214910e-02 ,
                            6.383713018105641179e-02 ,
                            5.155616403669016634e-02 ,
                            3.919159994094389976e-02 ,
                            2.676338990673743368e-02 ,
                            1.429166857403529889e-02 ,
                            1.796696044255141622e-03 ,
                            -1.070120033510124635e-02,
                            -2.318168475029436237e-02,
                            -3.562447177234751017e-02,
                            -4.800938478487482008e-02,
                            -6.031641350041105909e-02,
                            -7.252577008986405005e-02,
                            -8.461794346930706645e-02,
                            -9.657375131426171322e-02,
                            -1.083743894033946109e-01,
                            -1.200014779306022294e-01,
                            -1.314371044661173382e-01,
                            -1.426638632928318717e-01,
                            -1.536648908926760981e-01,
                            -1.644238974088296779e-01,
                            -1.749251939619214102e-01,
                            -1.851537157513418741e-01,
                            -1.950950409255596263e-01,
                            -2.047354052570615757e-01,
                            -2.140617127075308268e-01,
                            -2.230615420163986062e-01,
                            -2.317231494903906341e-01,
                            -2.400354682125307448e-01,
                            -2.479881039258757802e-01,
                            -2.555713278795914700e-01,
                            -2.627760669525980308e-01,
                            -2.695938913926929636e-01,
                            -2.760170005266717741e-01,
                            -2.820382068094793304e-01,
                            -2.876509185878604047e-01])

    nextTwoPositions = np.zeros(shape=(2, ))

    currentPos1 = targetPos[0]
    currentVel1 =  0
    currentAcc1 =  0

    currentPos2 = targetPos[0]
    currentVel2 =  0
    currentAcc2 =  0

    currentPos3 = targetPos[0]
    currentVel3 =  0
    currentAcc3 =  0

    sectionLength = rospy.Duration(0.01)
    tSection = rospy.Duration(0.0)
    dt = rospy.Duration(0.001)  # controller time steps
    stepsPerSection = int(round(sectionLength.to_sec() / dt.to_sec()))

    steps = int((len(targetPos) - 1) * sectionLength.to_sec() / dt.to_sec())

    # provide some space for coordinates (needed for plotting)
    pos1 = np.zeros(shape=(steps, ))
    vel1 = np.zeros(shape=(steps, ))
    acc1 = np.zeros(shape=(steps, ))
    
    pos2 = np.zeros(shape=(steps, ))
    vel2 = np.zeros(shape=(steps, ))
    acc2 = np.zeros(shape=(steps, ))
    
    pos3 = np.zeros(shape=(steps, ))
    vel3 = np.zeros(shape=(steps, ))
    acc3 = np.zeros(shape=(steps, ))
    
    tVec = np.zeros(shape=(steps, ))

    # poynomial coefficients
    coefs1 = np.zeros(shape=(polyOrder + 1, ))  # for nextVelocity = avg vel of next section
    coefs2 = np.zeros(shape=(polyOrder + 1, ))  # for nextVelocity = avg vel of second next section
    coefs3 = np.zeros(shape=(polyOrder + 1, ))  # for nextVelocity = avg vel of both next 2 sections
    
    t = rospy.Duration(0.0)     # start time

    tMeasured = np.zeros(shape=(steps, ))

    index = 1
    # loop which gets executed once every 1ms
    for j in range(steps):

        t00 = time.time()

        if j >= pos1.shape[0]:  # as all vectors have the same shape, checking only one is enough
            print("Exceeded storage space at t=",t)
            break

        if j == 0 or ((j-1) % stepsPerSection == 0 and j != 1):

            t0 = time.time()

            nextTwoPositions[0] = targetPos[index]
            if index < targetPos.shape[0]-1:
                nextTwoPositions[1] = targetPos[index + 1]
            else: # break down for last point -> create an imaginary point on same position
                nextTwoPositions[1] = targetPos[-1]

            nextVelocity1 = (nextTwoPositions[0]-targetPos[index - 1])/sectionLength.to_sec()
            nextVelocity2 = (nextTwoPositions[1]-nextTwoPositions[0])/sectionLength.to_sec()
            nextVelocity3 = (nextTwoPositions[1]-targetPos[index - 1])/(sectionLength.to_sec() * 2)

            index += 1

            coefs1 = calcCoefs(currentPos1, currentVel1, currentAcc1, 0, nextTwoPositions[0], nextVelocity1, 0, 0, sectionLength.to_sec())
            coefs2 = calcCoefs(currentPos2, currentVel2, currentAcc2, 0, nextTwoPositions[0], nextVelocity2, 0, 0, sectionLength.to_sec())
            coefs3 = calcCoefs(currentPos3, currentVel3, currentAcc3, 0, nextTwoPositions[0], nextVelocity3, 0, 0, sectionLength.to_sec())
            #print(coefs3)
            
            if j == 0:
                tSection = rospy.Duration(0.0)
            else:
                tSection = dt

            t1 = time.time()
            #print("New trajectories took {}s".format(t1-t0))
        
        currentPos1, currentVel1, currentAcc1 = evaluatePolynom(coef=coefs1, t=tSection.to_sec())
        currentPos2, currentVel2, currentAcc2 = evaluatePolynom(coef=coefs2, t=tSection.to_sec())
        currentPos3, currentVel3, currentAcc3 = evaluatePolynom(coef=coefs3, t=tSection.to_sec())
            
        pos1[j] = currentPos1
        vel1[j] = currentVel1
        acc1[j] = currentAcc1

        pos2[j] = currentPos2
        vel2[j] = currentVel2
        acc2[j] = currentAcc2

        pos3[j] = currentPos3
        vel3[j] = currentVel3
        acc3[j] = currentAcc3

        tVec[j] = t.to_sec()

        t += dt
        tSection += dt

        t11 = time.time()
        tMeasured[j] = t11-t00

    print("tMeasured max: {}s, avg: {}s, std:{}s".format(np.amax(tMeasured), np.mean(tMeasured), np.std(tMeasured)))

    targetT = np.zeros(shape=targetPos.shape)
    for i in range(targetT.shape[0]):
        targetT[i] = i * sectionLength.to_sec()

    fig, axs = plt.subplots(3, 3, sharex=True, sharey='row')
    labels = ["s in m", "v in m/s", "a in m/s2"]
    axs[0][0].plot(tVec, pos1)
    axs[0][0].plot(targetT, targetPos, 'kx')
    axs[0][0].set_ylabel(labels[0])
    axs[0][0].grid()
    axs[1][0].plot(tVec, vel1)
    axs[1][0].set_ylabel(labels[1])
    axs[1][0].grid()
    axs[2][0].plot(tVec, acc1)
    axs[2][0].set_ylabel(labels[2])
    axs[2][0].grid()
    axs[0][0].set_title("end velocity 1")

    axs[0][1].plot(tVec, pos2)
    axs[0][1].plot(targetT, targetPos, 'kx')
    axs[0][1].grid()
    axs[1][1].plot(tVec, vel2)
    axs[1][1].grid()
    axs[2][1].plot(tVec, acc2)
    axs[2][1].grid()
    axs[0][1].set_title("end velocity 2")

    axs[0][2].plot(tVec, pos3)
    axs[0][2].plot(targetT, targetPos, 'kx')
    axs[0][2].grid()
    axs[1][2].plot(tVec, vel3)
    axs[1][2].grid()
    axs[2][2].plot(tVec, acc3)
    axs[2][2].grid()
    axs[0][2].set_title("end velocity 3")

    if polyOrder == 7: # in this case we have to limit axes as polynom returns too high values for graph
        axs[0][0].set_ylim(bottom=0.14, top=0.22)
        axs[1][0].set_ylim(bottom=-4, top=4)
        axs[2][0].set_ylim(bottom=-1250, top=1250)
        
    fig.set_tight_layout(True)
    plt.show()


if __name__ == "__main__":
    main()