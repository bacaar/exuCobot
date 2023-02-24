#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 13.10.2022

Script for reading and plotting exudyn target and current robot positions
"""

import os

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

logPath = os.getcwd() + "/log/"

def capData(data, tmin, tmax):
    for i in range(data.shape[0]):
        if data["rt"][i] < tmin or data["rt"][i] > tmax:
            data = data.drop([i])
    return data

def main():

    exudynIC = pd.read_csv(logPath + "exudynIC.csv", header=0)
    targetIC = pd.read_csv(logPath + "targetIC.csv", header=0)
    currentPositionIC = pd.read_csv(logPath + "currentPositionIC.csv", header=0)

    exudynVC = pd.read_csv(logPath + "exudynVC.csv", header=0)
    targetVC = pd.read_csv(logPath + "targetVC.csv", header=0)
    currentPositionVC = pd.read_csv(logPath + "currentPositionVC.csv", header=0)

    # set both datasets to be on equilibrium position = 0
    targetIC["py"] -= exudynIC["py"][0]
    currentPositionIC["py"] -= exudynIC["py"][0]
    exudynIC["py"] -= exudynIC["py"][0]
    
    targetVC["py"] -= exudynVC["py"][0]
    currentPositionVC["py"] -= exudynVC["py"][0]
    exudynVC["py"] -= exudynVC["py"][0]

    # let all datasets start with zero (not ros global time)
    tminIC = min((exudynIC["rt"][0], targetIC["rt"][0], currentPositionIC["rt"][0]))
    exudynIC["rt"] -= tminIC
    targetIC["rt"] -= tminIC
    currentPositionIC["rt"] -= tminIC

    tminVC = min((exudynVC["rt"][0], targetVC["rt"][0], currentPositionVC["rt"][0]))
    exudynVC["rt"] -= tminVC
    targetVC["rt"] -= tminVC
    currentPositionVC["rt"] -= tminVC

    # synchronize IC and VC
    timedif = exudynVC.loc[exudynVC['py'].idxmax(), 'rt'] - exudynIC.loc[exudynIC['py'].idxmax(), 'rt']
    targetVC["rt"] -= timedif
    currentPositionVC["rt"] -= timedif
    exudynVC["rt"] -= timedif

    # define from when to when plot data
    tmin = 6.8
    tmax = 11

    exudynIC = capData(exudynIC, tmin, tmax)
    targetIC = capData(targetIC, tmin, tmax)
    targetVC = capData(targetVC, tmin, tmax)
    exudynVC = capData(exudynVC, tmin, tmax)
    currentPositionIC = capData(currentPositionIC, tmin, tmax)
    currentPositionVC = capData(currentPositionVC, tmin, tmax)
    
    # let time start at 0
    targetIC["rt"] -= tmin
    currentPositionIC["rt"] -= tmin
    exudynIC["rt"] -= tmin
    targetVC["rt"] -= tmin
    currentPositionVC["rt"] -= tmin
    exudynVC["rt"] -= tmin


    legendList = []

    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(16, 9))

    axs[0].plot(np.array(exudynIC["rt"]), np.array(exudynIC["py"]))
    axs[1].plot(np.array(exudynVC["rt"]), np.array(exudynVC["py"]))
    legendList.append("exudyn target")

    axs[0].plot(np.array(targetIC["rt"]), np.array(targetIC["py"]), "-")
    axs[1].plot(np.array(targetVC["rt"]), np.array(targetVC["py"]), "-")
    legendList.append("received target")

    axs[0].plot(np.array(currentPositionIC["rt"]), np.array(currentPositionIC["py"]))
    axs[1].plot(np.array(currentPositionVC["rt"]), np.array(currentPositionVC["py"]))
    legendList.append("robot position")    

    axs[0].grid()
    axs[1].grid()

    axs[0].set_title("Impedance Control")
    axs[1].set_title("Velocity Control")

    axs[0].set_ylabel("y pos in m")
    axs[1].set_ylabel("y pos in m")

    axs[1].set_xlabel("t in s")

    fig.legend(legendList)
    plt.show()

if __name__ == "__main__":
    main()