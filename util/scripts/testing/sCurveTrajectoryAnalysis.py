#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 08.10.2022

Script for testing creation of s-Curve trajectories
"""

import numpy as np
import rospy
import matplotlib.pyplot as plt
import time

useRealData = False
polyOrder = 5

v_max = 1.7
a_max = 13
j_max = 6500

class sCurve:

    def __init__(self):
        self.t = []
        
        self.v = 0
        self.a = 0
        self.j = 0

        self.s0 = 0
        self.v0 = 0
        self.a0 = 0
        self.sT = 0

    def calcTrajectory(self, s0, ds0, dds0, sT, dsT, ddsT, T):

        self.s0 = s0
        self.v0 = ds0
        self.a0 = dds0
        self.sT = sT
        
        # calculate trajectoy when using maximal acceleration and velocity
        d_s = abs(sT - s0)          # distance of travel
        t_a = v_max / a_max         # time needed to accelerate from 0 to max velocity (same as breaking time)
        t_total = d_s / v_max + t_a

        # t_total must not be bigger than T
        if t_total > T:
            print("ERROR: needed time for trajectory ({}) is bigger than permitted time({})".format(t_total, T))
            exit(-1)

        # scale trajectory so that it uses full time T
        factor = T / t_total

        #self.a = a_max / factor
        #self.v = v_max / factor

        self.a = a_max
        self.v = 1.091
        t_a = self.v / self.a

        print("new t total:", d_s / self.v + t_a)

        self.t.append(0)            # trajectory start time
        self.t.append(t_a)          # time when accelerating ends
        self.t.append((T - t_a))    # time when breaking starts
        self.t.append(T)            # trajectory end time


    def evaluateTrajectory(self, t):
        if t <= self.t[1]:
            a = self.a0 + self.a
            v = self.v0 + self.a * t
            s = self.s0 + 0.5 * self.a * t**2
            return np.array([s, v, a, 0])
        elif t < self.t[2]:
            a = 0
            v = self.v0 + self.v
            s = self.s0 + self.v*t - 0.5 * self.v**2 / self.a
            return np.array([s, v, a, 0])
        elif t <= self.t[3]:
            a = self.a0 - self.a
            v = self.v0 + self.a*(self.t[3] - t)
            s = self.s0 + self.v*(self.t[3] - self.t[1]) - self.a * 0.5 * (self.t[3]-t)**2
            return np.array([s, v, a, 0])
    


def main():

    if not useRealData:
        targetPos = np.array([0, 1])
        """targetPos = np.array([0.15,
                        0.15,
                        0.16,
                        0.17,
                        0.16,
                        0.18,
                        0.19,
                        0.19,
                        0.18])
        targetPos = np.array([#-0.0312898949951993,
                              -0.0312898949951993,
                              -0.031293220937771316,
                              -0.031337655689557375,
                              -0.031433736624773334,
                              -0.0315806816957106,
                              -0.0317781882662852,
                              -0.03203152771955142,
                              -0.03234145393735044])"""

    
    else:
        targetPos = np.array([4.769210092045206295e-02,
        4.595609288870805553e-02,
        4.415239596025011259e-02,
        4.228365097109687554e-02,
        4.035259600196117180e-02,
        3.836206248397966423e-02,
        3.631497114442761021e-02,
        3.421432779692457071e-02,
        3.206321898118513758e-02,
        2.986480745792929881e-02,
        2.762232756512017495e-02,
        2.533908044225485767e-02,
        2.301842912998552926e-02,
        2.066379355287450981e-02,
        1.827864539360979368e-02,
        1.586650286749158312e-02,
        1.343092540646395605e-02,
        1.097550826239124255e-02,
        8.503877039663265691e-03,
        6.019682167561879460e-03,
        3.526593323110560618e-03,
        1.028293815380831866e-03,
        -1.471525057578748630e-03,
        -3.969169667893623910e-03,
        -6.460949721702746729e-03,
        -8.943183916286345791e-03,
        -1.141220557910505029e-02,
        -1.386436827746617695e-02,
        -1.629605138768974726e-02,
        -1.870366561283165474e-02,
        -2.108365843830117825e-02,
        -2.343251951498670493e-02,
        -2.574678595988144636e-02,
        -2.802304756457552060e-02,
        -3.025795190242308585e-02,
        -3.244820932565573024e-02,
        -3.459059784420825956e-02,
        -3.668196787854616225e-02,
        -3.871924687931116438e-02,
        -4.069944380716217225e-02,
        -4.261965346673690647e-02,
        -4.447706068922352429e-02,
        -4.626894435858441845e-02,
        -4.799268127702194242e-02,
        -4.964574986580649885e-02,
        -5.122573369810257127e-02,
        -5.273032486092321669e-02,
        -5.415732714379939416e-02,
        -5.550465905219459373e-02,
        -5.677035664409446625e-02,
        -5.795257618855753634e-02,
        -5.904959664535813779e-02,
        -6.005982196513071081e-02,
        -6.098178320967595489e-02,
        -6.181414049230582464e-02,
        -6.255568473827266551e-02,
        -6.320533926545768288e-02,
        -6.376216118559985269e-02,
        -6.422534262640211544e-02,
        -6.459421177487856269e-02,
        -6.486823374232475459e-02,
        -6.504701125124667804e-02,
        -6.513028514455787565e-02,
        -6.511793471727855831e-02,
        -6.500997787090523339e-02])

    nextTwoPositions = np.zeros(shape=(2, ))

    sectionLength = 1    # seconds
    dt = 0.001  # controller time steps in seconds
    stepsPerSection = int(round(sectionLength / dt))

    steps = int((len(targetPos) - 1) * sectionLength / dt)

    currentPos1 = targetPos[0]
    #currentVel1 = (targetPos[1]-targetPos[0])/sectionLength
    currentVel1 = 0
    currentAcc1 = 0

    currentPos2 = targetPos[0]
    currentVel2 = (targetPos[1]-targetPos[0])/sectionLength
    currentAcc2 = 0

    currentPos3 = targetPos[0]
    currentVel3 = (targetPos[1]-targetPos[0])/sectionLength
    currentAcc3 = 0

    # provide some space for coordinates (needed for plotting)
    states1 = np.zeros(shape=(steps, 4))
    states2 = np.zeros(shape=(steps, 4))
    states3 = np.zeros(shape=(steps, 4))
    
    tVec = np.zeros(shape=(steps, ))
    
    t = 0           # global time
    tSection = 0    # section time

    tMeasured = np.zeros(shape=(steps, ))

    # s-curves
    curve1 = sCurve()   # for nextVelocity = avg vel of next section
    curve2 = sCurve()   # for nextVelocity = avg vel of second next section
    curve3 = sCurve()   # for nextVelocity = avg vel of both next 2 sections

    index = 1
    # loop which gets executed once every 1ms (later in practice)
    for j in range(steps):

        t00 = time.time()

        if j >= states1.shape[0]:  # as all vectors have the same shape, checking only one is enough
            print("Exceeded storage space at t=",t)
            break

        if j == 0 or ((j-1) % stepsPerSection == 0 and j != 1):

            t0 = time.time()

            nextTwoPositions[0] = targetPos[index]
            if index < targetPos.shape[0]-1:
                nextTwoPositions[1] = targetPos[index + 1]
            else: # break down for last point -> create an imaginary point on same position
                nextTwoPositions[1] = targetPos[-1]

            nextVelocity1 = (nextTwoPositions[0] - targetPos[index - 1]) / sectionLength
            nextVelocity2 = (nextTwoPositions[1] - nextTwoPositions[0]) / sectionLength
            nextVelocity3 = (nextTwoPositions[1] - targetPos[index - 1]) / (sectionLength * 2)

            nextAcceleration1 = (nextVelocity1 - currentVel1)/sectionLength
            nextAcceleration2 = (nextVelocity2 - currentVel2)/sectionLength
            nextAcceleration3 = (nextVelocity3 - currentVel3)/sectionLength

            index += 1

            curve1.calcTrajectory(currentPos1, currentVel1, currentAcc1, nextTwoPositions[0], nextVelocity1, nextAcceleration1, sectionLength)
            #curve2.calcTrajectory(currentPos2, currentVel2, currentAcc2, nextTwoPositions[0], nextVelocity2, nextAcceleration2, sectionLength)
            #curve3.calcTrajectory(currentPos3, currentVel3, currentAcc3, nextTwoPositions[0], nextVelocity3, nextAcceleration3, sectionLength)
            #print(coefs3)
            
            if j == 0:
                tSection = 0
            else:
                tSection = dt

            t1 = time.time()
            #print("New trajectories took {}s".format(t1-t0))
        
        states1[j] = curve1.evaluateTrajectory(tSection)
        #states2[j] = curve2.evaluateTrajectory(tSection)
        #states3[j] = curve3.evaluateTrajectory(tSection)

        tVec[j] = t

        t += dt
        tSection += dt

        t11 = time.time()
        tMeasured[j] = t11-t00

    print("tMeasured max: {}s, avg: {}s, std:{}s".format(np.amax(tMeasured), np.mean(tMeasured), np.std(tMeasured)))

    targetT = np.zeros(shape=targetPos.shape)
    for i in range(targetT.shape[0]):
        targetT[i] = i * sectionLength

    cropStart = 10
    cropEnd = 10

    fig, axs = plt.subplots(4, 3, sharex=True, sharey='row')
    labels = ["s in m", "v in m/s", "a in m/s2", "j in m/s3"]
    
    axs[0][0].plot(targetT, targetPos, 'kx')
    axs[0][0].set_ylabel(labels[0])
    axs[0][0].grid()
    axs[1][0].set_ylabel(labels[1])
    axs[1][0].grid()
    axs[2][0].set_ylabel(labels[2])
    axs[2][0].grid()
    axs[3][0].set_ylabel(labels[3])
    axs[3][0].grid()
    axs[3][0].set_xlabel("t in s")
    axs[0][0].set_title("v_T = v_avg1")

    axs[0][1].plot(targetT, targetPos, 'kx')
    axs[0][1].grid()
    axs[1][1].grid()
    axs[2][1].grid()
    axs[3][1].grid()
    axs[3][1].set_xlabel("t in s")
    axs[0][1].set_title("v_T = v_avg2")

    axs[0][2].plot(targetT, targetPos, 'kx')
    axs[0][2].grid()
    axs[1][2].grid()
    axs[2][2].grid()
    axs[3][2].grid()
    axs[3][2].set_xlabel("t in s")
    axs[0][2].set_title("v_T = v_avg12")

    if not useRealData:
        #axs[0][0].plot(tVec, pos1)
        #axs[1][0].plot(tVec, vel1)
        #axs[2][0].plot(tVec, acc1)
        #axs[3][0].plot(tVec, jerk1)
        #axs[0][1].plot(tVec, pos2)
        #axs[1][1].plot(tVec, vel2)
        #axs[2][1].plot(tVec, acc2)
        #axs[3][1].plot(tVec, jerk2)
        #axs[0][2].plot(tVec, pos3)
        #axs[1][2].plot(tVec, vel3)
        #axs[2][2].plot(tVec, acc3)
        #axs[3][2].plot(tVec, jerk3)

        for i in range(4):      # 4 rows
            axs[i][0].plot(tVec, states1[:,i])
            axs[i][1].plot(tVec, states2[:,i])
            axs[i][2].plot(tVec, states3[:,i])

    else:
        #axs[0][0].plot(tVec[cropStart:-cropEnd], pos1[cropStart:-cropEnd])
        #axs[1][0].plot(tVec[cropStart:-cropEnd], vel1[cropStart:-cropEnd])
        #axs[2][0].plot(tVec[cropStart:-cropEnd], acc1[cropStart:-cropEnd])
        #axs[3][0].plot(tVec[cropStart:-cropEnd], jerk1[cropStart:-cropEnd])

        #axs[0][1].plot(tVec[cropStart:-cropEnd], pos2[cropStart:-cropEnd])
        #axs[1][1].plot(tVec[cropStart:-cropEnd], vel2[cropStart:-cropEnd])
        #axs[2][1].plot(tVec[cropStart:-cropEnd], acc2[cropStart:-cropEnd])
        #axs[3][1].plot(tVec[cropStart:-cropEnd], jerk2[cropStart:-cropEnd])

        #axs[0][2].plot(tVec[cropStart:-cropEnd], pos3[cropStart:-cropEnd])
        #axs[1][2].plot(tVec[cropStart:-cropEnd], vel3[cropStart:-cropEnd])
        #axs[2][2].plot(tVec[cropStart:-cropEnd], acc3[cropStart:-cropEnd])
        #axs[3][2].plot(tVec[cropStart:-cropEnd], jerk3[cropStart:-cropEnd])

        for i in range(4):      # 4 rows
            axs[i][0].plot(tVec[cropStart:-cropEnd], states1[cropStart:-cropEnd,i])
            axs[i][1].plot(tVec[cropStart:-cropEnd], states2[cropStart:-cropEnd,i])
            axs[i][2].plot(tVec[cropStart:-cropEnd], states3[cropStart:-cropEnd,i])
            
        
    fig.set_tight_layout(True)
    plt.show()

    #plt.figure()
    #plt.plot(targetT, targetPos, 'bx')
    #plt.plot(targetT, exuTarget, 'r.')
    #plt.plot(tVec[cropStart:-cropEnd], pos3[cropStart:-cropEnd])
    #plt.show()


if __name__ == "__main__":
    main()