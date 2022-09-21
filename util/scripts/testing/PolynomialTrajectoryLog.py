#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 03.08.2022

Script for testing creation of polynomial trajectories
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from PolynomialTrajectoryAnalysis import evaluatePolynom, calcCoefs

def offlineCalculation(polyOrder=5):
    segments = pd.read_csv(os.getcwd() + "/log/trajectoryCreation.csv")    

    segments = segments.drop([0])

    steps = (segments.shape[0]-1) * 10

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

    print(segments)

    for i in range(1, segments.shape[0]):

        coefs1 = calcCoefs(segments["cpx"][i], segments["cvx"][i], segments["cax"][i], segments["npx"][i], segments["nvx"][i], segments["nax"][i], segments["dt"][i], polyOrder)
        coefs2 = calcCoefs(segments["cpy"][i], segments["cvy"][i], segments["cay"][i], segments["npy"][i], segments["nvy"][i], segments["nay"][i], segments["dt"][i], polyOrder)
        coefs3 = calcCoefs(segments["cpz"][i], segments["cvz"][i], segments["caz"][i], segments["npz"][i], segments["nvz"][i], segments["naz"][i], segments["dt"][i], polyOrder)

        t = 0
        j = (i-1)*10

        counter = 0

        while(t < segments["dt"][i]):
        
            pos1[j], vel1[j], acc1[j], jerk1[j] = evaluatePolynom(coef=coefs1, t=t)
            pos2[j], vel2[j], acc2[j], jerk2[j] = evaluatePolynom(coef=coefs2, t=t)
            pos3[j], vel3[j], acc3[j], jerk3[j] = evaluatePolynom(coef=coefs3, t=t)

            tVec[j] = segments["t"][i] + t

            t += 0.001
            j += 1
            counter += 1

    #print(np.mean(np.array(segments["cpx"])), np.std(np.array(segments["cpx"])))
    #print(np.mean(np.array(segments["npx"])), np.std(np.array(segments["npx"])))

    #print(np.mean(pos1), np.std(pos1))

    data = np.array([[pos1, vel1, acc1, jerk1],
                     [pos2, vel2, acc2, jerk2],
                     [pos3, vel3, acc3, jerk3]]).T

    return tVec, data


def main():
    t, data = offlineCalculation()

    fig, axs = plt.subplots(4, 3, sharex=True)
    labels = ["s in m", "v in m/s", "a in m/s2", "j in m/s3"]
    
    axs[0][0].set_ylabel(labels[0])
    axs[0][0].grid()
    axs[1][0].set_ylabel(labels[1])
    axs[1][0].grid()
    axs[2][0].set_ylabel(labels[2])
    axs[2][0].grid()
    axs[3][0].set_ylabel(labels[3])
    axs[3][0].grid()
    axs[3][0].set_xlabel("t in s")
    axs[0][0].set_title("x-axis")

    axs[0][1].grid()
    axs[1][1].grid()
    axs[2][1].grid()
    axs[3][1].grid()
    axs[3][1].set_xlabel("t in s")
    axs[0][1].set_title("y-axis")

    axs[0][2].grid()
    axs[1][2].grid()
    axs[2][2].grid()
    axs[3][2].grid()
    axs[3][2].set_xlabel("t in s")
    axs[0][2].set_title("z-axis")

    for i in range(4):
        for j in range(3):
            axs[i][j].plot(t, data[i][j])
        
    fig.set_tight_layout(True)
    plt.show()


if __name__ == "__main__":
    main()