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

#logPath = os.getcwd() + "/../../../log/"
logPath = os.getcwd() + "/log/"

def main():
    # load log files as csv
    exudyn = pd.read_csv(logPath + "exudyn.log", header=0)
    commands = pd.read_csv(logPath + "commands.log", header=0)
    evaluatedTrajectory = pd.read_csv(logPath + "evaluatedTrajectory.log", header=0)
    currentPosition = pd.read_csv(logPath + "currentPosition.log", header=0)
    trajectoryCreation = pd.read_csv(logPath + "trajectoryCreation.log", header=0)

    # create new column with time as floating point number
    commands["t"] = commands["s"] + commands["ns"]*1e-9
    evaluatedTrajectory["t"] = evaluatedTrajectory["s"] + evaluatedTrajectory["ns"]*1e-9
    currentPosition["t"] = currentPosition["s"] + currentPosition["ns"]*1e-9
    trajectoryCreation["t"] = trajectoryCreation["s"] + trajectoryCreation["ns"]*1e-9

    # adapt time from exudyn to controller time
    # not sure if this is correct so just leave it out for the moment
    #tmin = min((commands["t"][0], currentPosition["t"][0], evaluatedTrajectory["t"][0], trajectoryCreation["t"][0]))
    #exudyn["t"] -= exudyn["t"][0] - tmin

    # delete first row in state (written before calulcated)
    evaluatedTrajectory = evaluatedTrajectory.drop(labels=0, axis=0)

    fig, axs = plt.subplots(4, 4, sharex="all")
    legendList = []

    # plot exudyn target positions
    plotSymbol = "x"
    markerSize = 4
    #legendList.append("exudyn target")
    #axs[0][0].plot(np.array(exudyn["t"]), np.array(exudyn["globalX"]), plotSymbol, ms=markerSize)
    #axs[0][1].plot(np.array(exudyn["t"]), np.array(exudyn["globalY"]), plotSymbol, ms=markerSize)
    #axs[0][2].plot(np.array(exudyn["t"]), np.array(exudyn["globalZ"]), plotSymbol, ms=markerSize)
    #axs[0][3].plot([],[])
    #for i in range(1, 4):
    #    for j in range(4):
    #        axs[i][j].plot([],[])


    # plot current position
    legendList.append("current position")
    plotSymbol = "o"
    axs[0][0].plot(np.array(currentPosition["t"]), np.array(currentPosition["px"]), plotSymbol, ms=markerSize)
    axs[0][1].plot(np.array(currentPosition["t"]), np.array(currentPosition["py"]), plotSymbol, ms=markerSize)
    axs[0][2].plot(np.array(currentPosition["t"]), np.array(currentPosition["pz"]), plotSymbol, ms=markerSize)
    axs[0][3].plot([],[], plotSymbol, ms=markerSize)
    for i in range(1,4):
        for j in range(4):
            axs[i][j].plot([], [], plotSymbol, ms=markerSize)

    # plot evaluated trajectory states
    legendList.append("evaluated trajectory")
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

    # absolute values in last column
    vel = np.concatenate((np.expand_dims(np.array(evaluatedTrajectory["vx"]), axis=0), 
                          np.expand_dims(np.array(evaluatedTrajectory["vy"]), axis=0), 
                          np.expand_dims(np.array(evaluatedTrajectory["vz"]), axis=0)))
    acc = np.concatenate((np.expand_dims(np.array(evaluatedTrajectory["ax"]), axis=0), 
                          np.expand_dims(np.array(evaluatedTrajectory["ay"]), axis=0), 
                          np.expand_dims(np.array(evaluatedTrajectory["az"]), axis=0)))
    jerk = np.concatenate((np.expand_dims(np.array(evaluatedTrajectory["jx"]), axis=0), 
                           np.expand_dims(np.array(evaluatedTrajectory["jy"]), axis=0), 
                           np.expand_dims(np.array(evaluatedTrajectory["jz"]), axis=0)))

    velAbs = np.linalg.norm(vel, axis=0)
    accAbs = np.linalg.norm(acc, axis=0)
    jerkAbs = np.linalg.norm(jerk, axis=0)

    axs[0][3].plot([],[])
    axs[1][3].plot(np.array(evaluatedTrajectory["t"]), velAbs)
    axs[2][3].plot(np.array(evaluatedTrajectory["t"]), accAbs)
    axs[3][3].plot(np.array(evaluatedTrajectory["t"]), jerkAbs)

    # plot start and end positions of segments
    legendList.append('trajectory start state')
    plotSymbol = "x"
    axs[0][0].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cpx"]), plotSymbol, ms=markerSize)
    axs[1][0].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cvx"]), plotSymbol, ms=markerSize)
    axs[2][0].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cax"]), plotSymbol, ms=markerSize)
    axs[3][0].plot([],[])

    axs[0][1].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cpy"]), plotSymbol, ms=markerSize)
    axs[1][1].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cvy"]), plotSymbol, ms=markerSize)
    axs[2][1].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cay"]), plotSymbol, ms=markerSize)
    axs[3][1].plot([],[])

    axs[0][2].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cpz"]), plotSymbol, ms=markerSize)
    axs[1][2].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["cvz"]), plotSymbol, ms=markerSize)
    axs[2][2].plot(np.array(trajectoryCreation["t"]), np.array(trajectoryCreation["caz"]), plotSymbol, ms=markerSize)
    axs[3][2].plot([],[])

    legendList.append('trajectory end state')
    axs[0][0].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["npx"]), plotSymbol, ms=markerSize)
    axs[1][0].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nvx"]), plotSymbol, ms=markerSize)
    axs[2][0].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nax"]), plotSymbol, ms=markerSize)
    axs[3][0].plot([],[])

    axs[0][1].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["npy"]), plotSymbol, ms=markerSize)
    axs[1][1].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nvy"]), plotSymbol, ms=markerSize)
    axs[2][1].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nay"]), plotSymbol, ms=markerSize)
    axs[3][1].plot([],[])

    axs[0][2].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["npz"]), plotSymbol, ms=markerSize)
    axs[1][2].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["nvz"]), plotSymbol, ms=markerSize)
    axs[2][2].plot(np.array(trajectoryCreation["t"]+trajectoryCreation["dt"]), np.array(trajectoryCreation["naz"]), plotSymbol, ms=markerSize)
    axs[3][2].plot([],[])

    #for i in range(trajectoryCreation.shape[0]):
    #    axs[0][0].plot([trajectoryCreation["t"][i], trajectoryCreation["t"][i]+trajectoryCreation["dt"][i]], [trajectoryCreation["cpx"][i], trajectoryCreation["npx"][i]], "k")
    #    axs[0][1].plot([trajectoryCreation["t"][i], trajectoryCreation["t"][i]+trajectoryCreation["dt"][i]], [trajectoryCreation["cpy"][i], trajectoryCreation["npy"][i]], "k")
    #    axs[0][2].plot([trajectoryCreation["t"][i], trajectoryCreation["t"][i]+trajectoryCreation["dt"][i]], [trajectoryCreation["cpz"][i], trajectoryCreation["npz"][i]], "k")

    legendList.append("commanded velocity")
    axs[1][0].plot(np.array(commands["t"]), np.array(commands["vx"]), plotSymbol, ms=markerSize)
    axs[1][1].plot(np.array(commands["t"]), np.array(commands["vy"]), plotSymbol, ms=markerSize)
    axs[1][2].plot(np.array(commands["t"]), np.array(commands["vz"]), plotSymbol, ms=markerSize)
    axs[1][3].plot([],[], plotSymbol, ms=markerSize)

    for i in (0, 2, 3):
        for j in range(4):
            axs[i][j].plot([],[], plotSymbol, ms=markerSize)

    for j in range(4):
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
    axs[0][3].set_title("absolute")

    fig.legend(legendList)

    plt.show()

    # write trajectoryCreation

    # delete s and ns columns
    trajectoryCreation = trajectoryCreation.drop(columns=['s', 'ns'])

    # delete x and y axes
    yOnly = False
    if(yOnly):
        trajectoryCreation = trajectoryCreation.drop(columns=['cpx', 'cvx', 'cax', 'cpz', 'cvz', 'caz'])
        trajectoryCreation = trajectoryCreation.drop(columns=['npx', 'nvx', 'nax', 'npz', 'nvz', 'naz'])

    # create column tend
    trajectoryCreation["tend"] = trajectoryCreation["t"] + trajectoryCreation["dt"]

    # move t and tend column to front
    cols = trajectoryCreation.columns.tolist()
    cols = cols[-2:] + cols[:-2]
    trajectoryCreation = trajectoryCreation[cols]
    

    trajectoryCreation.to_csv(logPath + "trajectoryCreation.csv")


if __name__ == "__main__":
    main()