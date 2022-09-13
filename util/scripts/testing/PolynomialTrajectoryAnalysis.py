#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 03.08.2022

Script for testing creation of polynomial trajectories
"""

import numpy as np
import rospy
import matplotlib.pyplot as plt
import time


polyOrder = 5
useRealData = False

# polynoms only implemented for 3rd and 5th order
assert polyOrder == 3 or polyOrder == 5

def calcCoefs(s0, ds0, dds0, sT, dsT, ddsT, T):

    T2 = T*T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

    if polyOrder == 3:
        """
        # boundary conditions: pos and acc
        m = np.array([[0, 0, 0, 1],
                      [T3, T2, T, 1],
                      [0, 2, 0, 0],
                      [6*T, 2, 0, 0]])
                    
        vec = np.array([s0, sT, dds0, ddsT])
        """
        
        # boundary conditions: pos x2, vel x2
        m = np.array([[0, 0, 0, 1],
                      [T3, T2, T, 1],
                      [0, 0, 1, 0],
                      [3*T2, 2*T, 1, 0]])
        
        vec = np.array([s0, sT, ds0, dsT])                    

        solution = np.linalg.inv(m) @ vec
        return solution

    if polyOrder == 5:

        # boundary conditions: pos x2, vel x2, acc x2
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

    if polyOrder == 3:

        A, B, C, D = coef

        t2 = t * t
        t3 = t2 * t

        s = A*t3 + B*t2 + C*t + D
        v = 3*A*t2 + 2*B*t + C
        a = 6*A*t + 2*B
        j = 6*A

        return np.array([s, v, a, j])

    if polyOrder == 5:

        A, B, C, D, E, F = coef

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        s = A*t5 + B*t4 + C*t3 + D*t2 + E*t + F
        v = 5*A*t4 + 4*B*t3 + 3*C*t2 + 2*D*t + E
        a = 20*A*t3 + 12*B*t2 + 6*C*t + 2*D
        j = 60*A*t2 + 24*B*t + 6*C

        return np.array([s, v, a, j])


def main():
    rospy.init_node('PolynomialTrajectoryTest', anonymous=True)

    if not useRealData:
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
        -6.500997787090523339e-02])#,
        """
        -6.480657109051879416e-02,
        -6.450800924463062636e-02,
        -6.411472520765792993e-02,
        -6.362728930484973233e-02,
        -6.304640857940468912e-02,
        -6.237292588146825523e-02,
        -6.160781877865129097e-02,
        -6.075219828769617969e-02,
        -5.980730742693074475e-02,
        -5.877451958917845332e-02,
        -5.765533673487799327e-02,
        -5.645138740525479371e-02,
        -5.516442455554848578e-02,
        -5.379632320846927662e-02,
        -5.234907792827936390e-02,
        -5.082480011616308246e-02,
        -4.922571512784013059e-02,
        -4.755415921472083696e-02,
        -4.581257629027679634e-02,
        -4.400351452370521166e-02,
        -4.212962276342002710e-02,
        -4.019364679335812873e-02,
        -3.819842542560381027e-02,
        -3.614688643333829887e-02,
        -3.404204232867424906e-02,
        -3.188698599046424498e-02,
        -2.968488614775122159e-02,
        -2.743898272506584135e-02,
        -2.515258205635007016e-02,
        -2.282905197482065773e-02,
        -2.047181678662290949e-02,
        -1.808435213663739383e-02,
        -1.567017977529649997e-02,
        -1.323286223571118470e-02,
        -1.077599743084745043e-02,
        -8.303213180864243981e-03,
        -5.818161681067746116e-03,
        -3.324513921231897484e-03,
        -8.259540672697074370e-04,
        1.673826183560933600e-03,
        4.171133262610093873e-03,
        6.662277393558957428e-03,
        9.143578248504202755e-03,
        1.161137058503680652e-02,
        1.406200985304384421e-02,
        1.649187776011151740e-02,
        1.889738778461713675e-02,
        2.127499062585780365e-02,
        2.362117958086751290e-02,
        2.593249583793744151e-02,
        2.820553367724132343e-02,
        3.043694556941300977e-02,
        3.262344716338483330e-02,
        3.476182215529444175e-02,
        3.684892703079079013e-02,
        3.888169567360322265e-02,
        4.085714383379179804e-02,
        4.277237344965278965e-02,
        4.462457681781195618e-02,
        4.641104060659251296e-02,
        4.812914970829085259e-02,
        4.977639092652486053e-02,
        5.135035649533004065e-02,
        5.284874742716894058e-02,
        5.426937668748421650e-02,
        5.561017219385355759e-02,
        5.686917963820614652e-02,
        5.804456513092293157e-02,
        5.913461766596694869e-02,
        6.013775140647970030e-02,
        6.105250779052417975e-02,
        6.187755745686129849e-02,
        6.261170199082277676e-02,
        6.325387549046157520e-02,
        6.380314595326930238e-02,
        6.425871648379688494e-02,
        6.461992632255308955e-02,
        6.488625169654693714e-02,
        6.505730649182250858e-02,
        6.513284274827912945e-02,
        6.511275097701318959e-02,
        6.499706030033558513e-02,
        6.478593841453539337e-02,
        6.447969137537201689e-02,
        6.407876320618677290e-02,
        6.358373532844785458e-02,
        6.299532581446198876e-02,
        6.231438846193826375e-02,
        6.154191169004274986e-02,
        6.067901725657376399e-02,
        5.972695879589140500e-02,
        5.868712017728594521e-02,
        5.756101368353183645e-02,
        5.635027800949521382e-02,
        5.505667608079856112e-02,
        5.368209269273982454e-02,
        5.222853196988030966e-02,
        5.069811464698659176e-02,
        4.909307517230621087e-02,
        4.741575863450975259e-02,
        4.566861751499562416e-02,
        4.385420826768327007e-02,
        4.197518772885000526e-02,
        4.003430936005458740e-02,
        3.803441932767748224e-02,
        3.597845242314090530e-02,
        3.386942782839996724e-02,
        3.171044473184969625e-02,
        2.950467780035348486e-02,
        2.725537251364806846e-02,
        2.496584036794502470e-02,
        2.263945395608812916e-02,
        2.027964193216036382e-02,
        1.788988386894330151e-02,
        1.547370501711964330e-02,
        1.303467097556265664e-02,
        1.057638228247759216e-02,
        8.102468937539963889e-03,
        5.616584865510421132e-03,
        3.122402332094820210e-03,
        6.236063230571353699e-04,
        -1.876111102224742311e-03,
        -4.373056471442149373e-03,
        -6.863540562882963769e-03,
        -9.343884060399654246e-03,
        -1.181042318845682004e-02,
        -1.425951531599078947e-02,
        -1.668754451798259986e-02,
        -1.909092708384529935e-02,
        -2.146611696199851593e-02,
        -2.380961113031032639e-02,
        -2.611795488244927910e-02,
        -2.838774702058310950e-02,
        -3.061564494531099356e-02,
        -3.279836963417115392e-02])"""
    
    """
    controllerData = np.load("controllerTarget.npy",allow_pickle=True)
    targetPos = controllerData[:,2]
    print(targetPos.shape)
    """

    nextTwoPositions = np.zeros(shape=(2, ))

    sectionLength = rospy.Duration(0.01)
    tSection = rospy.Duration(0.0)
    dt = rospy.Duration(0.001)  # controller time steps
    stepsPerSection = int(round(sectionLength.to_sec() / dt.to_sec()))

    steps = int((len(targetPos) - 1) * sectionLength.to_sec() / dt.to_sec())

    currentPos1 = targetPos[0]
    currentVel1 = (targetPos[1]-targetPos[0])/sectionLength.to_sec()
    currentAcc1 = 0

    currentPos2 = targetPos[0]
    currentVel2 = (targetPos[1]-targetPos[0])/sectionLength.to_sec()
    currentAcc2 = 0

    currentPos3 = targetPos[0]
    currentVel3 = (targetPos[1]-targetPos[0])/sectionLength.to_sec()
    currentAcc3 = 0

    # provide some space for coordinates (needed for plotting)
    pos1 = np.zeros(shape=(steps, ))
    vel1 = np.zeros(shape=(steps, ))
    acc1 = np.zeros(shape=(steps, ))
    jerk1 = np.zeros(shape=(steps, ))
    
    pos2 = np.zeros(shape=(steps, ))
    vel2 = np.zeros(shape=(steps, ))
    acc2 = np.zeros(shape=(steps, ))
    jerk2 = np.zeros(shape=(steps, ))
    
    pos3 = np.zeros(shape=(steps, ))
    vel3 = np.zeros(shape=(steps, ))
    acc3 = np.zeros(shape=(steps, ))
    jerk3 = np.zeros(shape=(steps, ))
    
    tVec = np.zeros(shape=(steps, ))

    # poynomial coefficients
    coefs1 = np.zeros(shape=(polyOrder + 1, ))  # for nextVelocity = avg vel of next section
    coefs2 = np.zeros(shape=(polyOrder + 1, ))  # for nextVelocity = avg vel of second next section
    coefs3 = np.zeros(shape=(polyOrder + 1, ))  # for nextVelocity = avg vel of both next 2 sections
    
    t = rospy.Duration(0.0)     # start time

    tMeasured = np.zeros(shape=(steps, ))

    index = 1
    # loop which gets executed once every 1ms (later in practice)
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

            nextVelocity1 = (nextTwoPositions[0] - targetPos[index - 1]) / sectionLength.to_sec()
            nextVelocity2 = (nextTwoPositions[1] - nextTwoPositions[0]) / sectionLength.to_sec()
            nextVelocity3 = (nextTwoPositions[1] - targetPos[index - 1]) / (sectionLength.to_sec() * 2)

            nextAcceleration1 = (nextVelocity1 - currentVel1)/sectionLength.to_sec()
            nextAcceleration2 = (nextVelocity2 - currentVel2)/sectionLength.to_sec()
            nextAcceleration3 = (nextVelocity3 - currentVel3)/sectionLength.to_sec()

            index += 1

            coefs1 = calcCoefs(currentPos1, currentVel1, currentAcc1, nextTwoPositions[0], nextVelocity1, nextAcceleration1, sectionLength.to_sec())
            coefs2 = calcCoefs(currentPos2, currentVel2, currentAcc2, nextTwoPositions[0], nextVelocity2, nextAcceleration2, sectionLength.to_sec())
            coefs3 = calcCoefs(currentPos3, currentVel3, currentAcc3, nextTwoPositions[0], nextVelocity3, nextAcceleration3, sectionLength.to_sec())
            #print(coefs3)
            
            if j == 0:
                tSection = rospy.Duration(0.0)
            else:
                tSection = dt

            t1 = time.time()
            #print("New trajectories took {}s".format(t1-t0))
        
        currentPos1, currentVel1, currentAcc1, currentJerk1 = evaluatePolynom(coef=coefs1, t=tSection.to_sec())
        currentPos2, currentVel2, currentAcc2, currentJerk2  = evaluatePolynom(coef=coefs2, t=tSection.to_sec())
        currentPos3, currentVel3, currentAcc3, currentJerk3  = evaluatePolynom(coef=coefs3, t=tSection.to_sec())
            
        pos1[j] = currentPos1
        vel1[j] = currentVel1
        acc1[j] = currentAcc1
        jerk1[j] = currentJerk1

        pos2[j] = currentPos2
        vel2[j] = currentVel2
        acc2[j] = currentAcc2
        jerk2[j] = currentJerk2

        pos3[j] = currentPos3
        vel3[j] = currentVel3
        acc3[j] = currentAcc3
        jerk3[j] = currentJerk3

        tVec[j] = t.to_sec()

        t += dt
        tSection += dt

        t11 = time.time()
        tMeasured[j] = t11-t00

    print("tMeasured max: {}s, avg: {}s, std:{}s".format(np.amax(tMeasured), np.mean(tMeasured), np.std(tMeasured)))

    targetT = np.zeros(shape=targetPos.shape)
    for i in range(targetT.shape[0]):
        targetT[i] = i * sectionLength.to_sec()

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
        axs[0][0].plot(tVec, pos1)
        axs[1][0].plot(tVec, vel1)
        axs[2][0].plot(tVec, acc1)
        axs[3][0].plot(tVec, jerk1)
        axs[0][1].plot(tVec, pos2)
        axs[1][1].plot(tVec, vel2)
        axs[2][1].plot(tVec, acc2)
        axs[3][1].plot(tVec, jerk2)
        axs[0][2].plot(tVec, pos3)
        axs[1][2].plot(tVec, vel3)
        axs[2][2].plot(tVec, acc3)
        axs[3][2].plot(tVec, jerk3)
    else:
        axs[0][0].plot(tVec[cropStart:-cropEnd], pos1[cropStart:-cropEnd])
        axs[1][0].plot(tVec[cropStart:-cropEnd], vel1[cropStart:-cropEnd])
        axs[2][0].plot(tVec[cropStart:-cropEnd], acc1[cropStart:-cropEnd])
        axs[3][0].plot(tVec[cropStart:-cropEnd], jerk1[cropStart:-cropEnd])

        axs[0][1].plot(tVec[cropStart:-cropEnd], pos2[cropStart:-cropEnd])
        axs[1][1].plot(tVec[cropStart:-cropEnd], vel2[cropStart:-cropEnd])
        axs[2][1].plot(tVec[cropStart:-cropEnd], acc2[cropStart:-cropEnd])
        axs[3][1].plot(tVec[cropStart:-cropEnd], jerk2[cropStart:-cropEnd])

        axs[0][2].plot(tVec[cropStart:-cropEnd], pos3[cropStart:-cropEnd])
        axs[1][2].plot(tVec[cropStart:-cropEnd], vel3[cropStart:-cropEnd])
        axs[2][2].plot(tVec[cropStart:-cropEnd], acc3[cropStart:-cropEnd])
        axs[3][2].plot(tVec[cropStart:-cropEnd], jerk3[cropStart:-cropEnd])
        
    fig.set_tight_layout(True)
    plt.show()

    #plt.figure()
    #plt.plot(targetT, targetPos, 'bx')
    #plt.plot(targetT, exuTarget, 'r.')
    #plt.plot(tVec[cropStart:-cropEnd], pos3[cropStart:-cropEnd])
    #plt.show()


if __name__ == "__main__":
    main()