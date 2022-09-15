#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 14.09.2022

Script for reading and plotting logfiles from state and segements
"""

import os

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

logPath = os.getcwd() + "/../../../log/"

def main():
    # load log files as csv
    exudyn = pd.read_csv(logPath + "exudyn.log", header=0)
    evaluatedTrajectory = pd.read_csv(logPath + "evaluatedTrajectory.log", header=0)
    currentPosition = pd.read_csv(logPath + "currentPosition.log", header=0)
    trajectoryCreation = pd.read_csv(logPath + "trajectoryCreation.log", header=0)

    # create new column with time as floating point number
    evaluatedTrajectory["t"] = evaluatedTrajectory["s"] + evaluatedTrajectory["ns"]*1e-9
    currentPosition["t"] = currentPosition["s"] + currentPosition["ns"]*1e-9
    trajectoryCreation["t"] = trajectoryCreation["s"] + trajectoryCreation["ns"]*1e-9

    # find earliest time stamp and substract it from every one to start from 0
    tmin = min(min(exudyn["t"][0], currentPosition["t"][0]), min(evaluatedTrajectory["t"][0], trajectoryCreation["t"][0]))
    exudyn["t"] -= tmin #- 0.02     # compensate programmed 80ms delay
    evaluatedTrajectory["t"] -= tmin
    currentPosition["t"] -= tmin
    trajectoryCreation["t"] -= tmin

    # delete first row in state (written before calulcated)
    evaluatedTrajectory = evaluatedTrajectory.drop(labels=0, axis=0)

    fig, axs = plt.subplots(4, 3, sharex="all")

    # plot exudyn target positions
    plotSymbol = "x"
    markerSize = 4
    axs[0][0].plot(np.array(exudyn["t"]), np.array(exudyn["globalX"]), plotSymbol, ms=markerSize)
    axs[0][1].plot(np.array(exudyn["t"]), np.array(exudyn["globalY"]), plotSymbol, ms=markerSize)
    axs[0][2].plot(np.array(exudyn["t"]), np.array(exudyn["globalZ"]), plotSymbol, ms=markerSize)

    # plot current position
    plotSymbol = "-"
    axs[0][0].plot(np.array(currentPosition["t"]), np.array(currentPosition["px"]), plotSymbol, ms=markerSize)
    axs[0][1].plot(np.array(currentPosition["t"]), np.array(currentPosition["py"]), plotSymbol, ms=markerSize)
    axs[0][2].plot(np.array(currentPosition["t"]), np.array(currentPosition["pz"]), plotSymbol, ms=markerSize)

    # plot empty arrays just that legend is correct for each subplot
    for i in range(1,4):
        for j in range(3):
            axs[i][j].plot([], [])
            axs[i][j].plot([], [])

    # plot evaluated trajectory states
    plotSymbol = "-"
    axs[0][0].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["px"]), plotSymbol)
    axs[1][0].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["vx"]), plotSymbol)
    axs[2][0].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["ax"]), plotSymbol)
    axs[3][0].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["jx"]), plotSymbol)

    axs[0][1].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["py"]), plotSymbol)
    axs[1][1].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["vy"]), plotSymbol)
    axs[2][1].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["ay"]), plotSymbol)
    axs[3][1].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["jy"]), plotSymbol)

    axs[0][2].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["pz"]), plotSymbol)
    axs[1][2].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["vz"]), plotSymbol)
    axs[2][2].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["az"]), plotSymbol)
    axs[3][2].plot(np.array(evaluatedTrajectory["t"]), np.array(evaluatedTrajectory["jz"]), plotSymbol)

    # plot start and end positions of segments
    plotSymbol = "x"
    axs[0][0].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cpx"]), plotSymbol, ms=markerSize)
    axs[1][0].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cvx"]), plotSymbol, ms=markerSize)
    axs[2][0].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cax"]), plotSymbol, ms=markerSize)

    axs[0][0].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["npx"]), plotSymbol, ms=markerSize)
    axs[1][0].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nvx"]), plotSymbol, ms=markerSize)
    axs[2][0].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nax"]), plotSymbol, ms=markerSize)

    axs[0][1].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cpy"]), plotSymbol, ms=markerSize)
    axs[1][1].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cvy"]), plotSymbol, ms=markerSize)
    axs[2][1].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cay"]), plotSymbol, ms=markerSize)

    axs[0][1].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["npy"]), plotSymbol, ms=markerSize)
    axs[1][1].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nvy"]), plotSymbol, ms=markerSize)
    axs[2][1].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nay"]), plotSymbol, ms=markerSize)

    axs[0][2].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cpz"]), plotSymbol, ms=markerSize)
    axs[1][2].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cvz"]), plotSymbol, ms=markerSize)
    axs[2][2].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["caz"]), plotSymbol, ms=markerSize)

    axs[0][2].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["npz"]), plotSymbol, ms=markerSize)
    axs[1][2].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nvz"]), plotSymbol, ms=markerSize)
    axs[2][2].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["naz"]), plotSymbol, ms=markerSize)

    for i in range(trajectoryCreation.shape[0]):
        axs[0][0].plot([trajectoryCreation["t"][i], trajectoryCreation["t"][i]+trajectoryCreation["dt"][i]], [trajectoryCreation["cpx"][i], trajectoryCreation["npx"][i]], "k")
        axs[0][1].plot([trajectoryCreation["t"][i], trajectoryCreation["t"][i]+trajectoryCreation["dt"][i]], [trajectoryCreation["cpy"][i], trajectoryCreation["npy"][i]], "k")
        axs[0][2].plot([trajectoryCreation["t"][i], trajectoryCreation["t"][i]+trajectoryCreation["dt"][i]], [trajectoryCreation["cpz"][i], trajectoryCreation["npz"][i]], "k")

    for j in range(3):
        axs[3][j].set_xlabel("t in s")
        for i in range(4):
            axs[i][j].grid()

    axs[0][0].set_ylabel("pos in m")
    axs[1][0].set_ylabel("vel in m/s")
    axs[2][0].set_ylabel("acc in m/s2")
    axs[3][0].set_ylabel("jerk in m/s3")

    axs[0][0].set_title("x-axis")
    axs[0][1].set_title("y-axis")
    axs[0][2].set_title("z-axis")

    fig.legend(['exudyn target', 'current Position', 'evaluated Trajectory', 'trajectory start state', 'trajectory end state'])

    plt.show()

    # write trajectoryCreation

    # delete s and ns columns
    trajectoryCreation = trajectoryCreation.drop(columns=['s', 'ns'])

    # create column tend
    trajectoryCreation["tend"] = trajectoryCreation["t"] + trajectoryCreation["dt"]

    # move t and tend column to front
    cols = trajectoryCreation.columns.tolist()
    cols = cols[-2:] + cols[:-2]
    trajectoryCreation = trajectoryCreation[cols]

    trajectoryCreation.to_csv(logPath + "trajectoryCreation.csv")


if __name__ == "__main__":
    main()