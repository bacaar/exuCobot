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

from PolynomialTrajectoryAnalysis import evaluatePolynom, calcCoefs


def main():

    path = "util/measurementData/latencyBetweenExudynAndController/"
    exuData = np.load(path + "exudynTarget20ms.npy")
    controllerData = np.load(path + "controllerTarget20ms.npy")

    # plot for first overview
    fix, ax = plt.subplots(1,1)
    ax.plot(exuData[:,0], exuData[:,2], "bx", label="sent by exudyn")
    ax.plot(controllerData[:,0], controllerData[:,2], "r.", label="received by controller")
    ax.legend()
    ax.set_xlabel("t in s")
    ax.set_ylabel("pos in m")
    ax.grid()

    # analyze time steps inbetween data points
    for data in (exuData[:,0], controllerData[:,0]):
        dtlist = np.zeros(shape=(data.shape[0]-1, ))
        for i in range(len(data)-1):
            dt = data[i+1] - data[i]
            dtlist[i] = dt
        print("Timesteps: datasize: {}  min: {:.8f}  max: {:.8f}  mean: {:.8f}  std: {:.8f}".format(dtlist.shape[0], np.amin(dtlist), np.amax(dtlist), np.mean(dtlist), np.std(dtlist)))

    plt.show()



if __name__ == "__main__":
    main()