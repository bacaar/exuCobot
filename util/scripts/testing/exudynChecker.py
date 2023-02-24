#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 04.10.2022

Script for analyzing exudyn commanded positions
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import os

def exudynCsv():

    logPath = os.getcwd() + "/log/"
    exudyn = pd.read_csv(logPath + "exudyn.csv", header=0)

    cvel = 0      # current velocity
    cacc = 0      # current velocity

    t = 0

    tVec = np.zeros(shape=(exudyn.shape[0]-2, ))
    stateVec = np.zeros(shape=(exudyn.shape[0]-2, 6))

    changeVec = np.zeros(shape=(exudyn.shape[0]-2, 4))

    for i in range(exudyn.shape[0]-2):

        cpos = exudyn["globalY"][i]       # current position
        npos = exudyn["globalY"][i+1]     # next position
        npos2 = exudyn["globalY"][i+2]    # second next position
        dt = exudyn["dt"][i+1]               # planned duration for reaching next (i+1) position
        dt2 = exudyn["dt"][i+2]              # planned duration for reaching second next (i+1) position

        nvel = (npos2 - cpos) /(dt+dt2)
        nacc = (nvel - cvel) / dt

        stateVec[i] = np.array([cpos, cvel, cacc, npos, nvel, nacc])
        tVec[i] = t
        t += dt

        changeVec[i][0] = npos - cpos   # dp
        changeVec[i][1] = npos2 - cpos  # dp2
        changeVec[i][2] = nvel - cvel   # dv
        changeVec[i][3] = nacc - cacc   # da

        # update vel and acc for next iteration
        cvel = nvel
        cacc = nacc

    fig, axs = plt.subplots(3, 2, sharex=True)

    for i in range(3):
        axs[i][0].plot(tVec, stateVec[:,i+3],'x')   # plot only next positions, current positions not relevant or redundant
        axs[i][0].grid()
        axs[i][1].grid()

    plot = [0, 0, 1, 2]

    for i in range(4):
        axs[plot[i]][1].plot(tVec, changeVec[:,i], "x")

    axs[0][0].set_title("absolute")
    axs[0][1].set_title("relative (change to last one)")
    axs[0][0].set_ylabel("pos in m")
    axs[1][0].set_ylabel("vel in m/s")
    axs[2][0].set_ylabel("acc in m/s2")
    axs[2][0].set_xlabel("t in s")
    axs[2][1].set_xlabel("t in s")

    plt.show()


def trajectoryCreation2Csv():
    # analyzes not all sent positions (as in exudynCsv()) but only those who where used by controller
    # before crash

    logPath = os.getcwd() + "/log/"
    df = pd.read_csv(logPath + "trajectoryCreation2.csv", header=0)

    t =    np.array(df["t"])
    cpy =  np.array(df["cpy"])
    npy =  np.array(df["npy"])
    npy2 = np.array(df["npy2"])
    cvy =  np.array(df["cvy"])
    nvy =  np.array(df["nvy"])
    cay =  np.array(df["cay"])
    nay =  np.array(df["nay"])
    dpy =  np.array(df["dpy"])
    dpy2 = np.array(df["dpy2"])
    dvy =  np.array(df["dvy"])
    day =  np.array(df["day"])
    dt =   np.array(df["dt"])

    ddp = dpy2 - dpy

    fig, axs = plt.subplots(3, 2, sharex=True)

    axs[0][0].plot(t, cpy, 'x')
    axs[0][0].plot(t, npy, 'x')
    axs[0][0].plot(t, npy2, 'x')
    axs[0][0].legend(("cpy", "npy", "npy2"))
    axs[0][0].grid()

    axs[1][0].plot(t, cvy, 'x')
    axs[1][0].plot(t, nvy, 'x')
    axs[1][0].legend(("cvy", "nvy"))
    axs[1][0].grid()

    axs[2][0].plot(t, cay, 'x')
    axs[2][0].plot(t, nay, 'x')
    axs[2][0].legend(("cay", "nay"))
    axs[2][0].grid()

    axs[0][1].plot(t, dpy, 'x')
    axs[0][1].plot(t, dpy2, 'x')
    axs[0][1].plot(t, ddp, 'x')
    axs[0][1].legend(("dpy", "dpy2", "dpy2 - dpy"))
    axs[0][1].grid()

    axs[1][1].plot(t, dvy, 'x')
    axs[1][1].legend(["dvy"])
    axs[1][1].grid()

    axs[2][1].plot(t, day, 'x')
    axs[2][1].legend(["day"])
    axs[2][1].grid()


    plt.show()

if __name__ == "__main__":
    #exudynCsv()
    trajectoryCreation2Csv()