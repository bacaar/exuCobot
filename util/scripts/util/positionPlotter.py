#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 10.10.2022

Script for reading and plotting positions (cartesian and joint space)
"""

import os

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

logPath = os.getcwd() + "/log/"

def main():
    # load log files as csv
    df = pd.read_csv(logPath + "currentPosition.csv", header=0)

    t = np.array(df["t"])

    p = np.empty(shape=(3, t.shape[0]))
    q = np.empty(shape=(7, t.shape[0]))

    coords = ["x", "y", "z"]

    for i in range(3):
        columnName = "p" + coords[i]
        p[i] = np.array(df[columnName])

    for i in range(7):
        columnName = "q" + str(i)
        q[i] = np.array(df[columnName])

    fig, axs = plt.subplots(7,2, sharex=True)

    plotSymbol = "x"
    markerSize = 4

    axs[0][0].set_title("cartesian EEF positions")
    axs[0][1].set_title("joint positions")

    axs[6][0].set_xlabel("t in s")
    axs[6][1].set_xlabel("t in s")

    for i in range(3):
        axs[i][0].plot(t,p[i],plotSymbol,ms=markerSize)
        axs[i][0].set_ylabel(coords[i] + " in m")
        axs[i][0].grid()

    for i in range(7):
        axs[i][1].plot(t,q[i],plotSymbol,ms=markerSize)
        axs[i][1].grid()

    plt.show()


if __name__ == "__main__":
    main()