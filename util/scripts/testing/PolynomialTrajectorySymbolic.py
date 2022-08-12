#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 03.08.2022

Script for testing creation of polynomial trajectories according to PR Angewandte Robotik (WS2019)
"""

import numpy as np
import sympy as sp

import matplotlib.pyplot as plt


def symbolicInverse():
    A, B, C, D, E, F, T = sp.symbols("A B C D E F T")

    """
    # order 3

    matrix = sp.Matrix([[0, 0, 0, 1],
                        [T**3, T**2, T, 1],
                        [0, 2, 0, 0],
                        [6*T, 2, 0, 0]])

    sp.init_printing()
    sp.pprint(matrix)

    print("\n")
    sp.pprint(matrix.inv())
    """

    # order 5
    matrix = sp.Matrix([[0, 0, 0, 0, 0, 1],
                       [sp.Pow(T, 5), sp.Pow(T, 4), sp.Pow(T, 3), sp.Pow(T, 2), T, 1],
                       [0, 0, 0, 0, 1, 0],
                       [5*sp.Pow(T, 4), 4*sp.Pow(T, 3), 3*sp.Pow(T, 2), 2*T, 1, 0],
                       [0, 0, 0, 2, 0, 0],
                       [20*sp.Pow(T, 3), 12*sp.Pow(T, 2), 6*T, 2, 0, 0]])

    #sp.pprint(matrix)

    print("\n")
    inv = matrix.inv()
    print()
    sp.pprint(inv)

    s0, sT, ds0, dsT, dds0, ddsT = sp.symbols("s0 sT ds0 dsT dds0 ddsT")

    vec = sp.Matrix([s0, sT, ds0, dsT, dds0, ddsT])
    sp.pprint(vec)
    print()
    sol = inv * vec
    sp.pprint(sol)
    print(sol)


def calcCoefs(s0, ds0, dds0, sT, dsT, ddsT, T):

    T2 = T*T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

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
    s = 0
    v = 0
    a = 0

    dCoef = np.array([5, 4, 3, 2, 1, 0])
    ddCoef = np.array([20, 12, 6, 2, 0, 0])

    for i in range(6):
        s += coef[-(i+1)] * t**i

        if i >= 1:
            v += dCoef[-(i+1)] * coef[-(i+1)] * t**(i-1)

        if i >= 2:
            a += ddCoef[-(i+1)] * coef[-(i+1)] * t**(i-2)

    return np.array([s, v, a])

def main():
    default = False
    if default:
        s0 = 0
        ds0 = 0
        dds0 = 0
        sT = 0.01
        dsT = 0
        ddsT = 0
        T = 0.01
    else: 
        # s0  0.0802262  ds0  -0.569798  dds0  0  sT  0.0756482  dsT  -0.991687  ddsT  0
        s0 = 0.0802262
        ds0 = -0.569798
        dds0 = 0
        sT = 0.0756482
        dsT = -0.991687
        ddsT = 0
        T = 0.01

    ret = calcCoefs(s0, ds0, dds0, sT, dsT, ddsT, T)
    #ret = [-300000000.0000002, 7000000.000000006, -40000.00000000004, 0.0, 0.0, 0.15]
    #ret = [-7.373455446213484e-07, 1.9885192159563303e-08, -1.1095835361629725e-10, -4.547473508864641e-13, 1.0000000000000036, 0.15]
    #ret = [1.12087e+08,   -2.55763e+06,   13790.4,   3.65609e-322,   -1.32768,   0.0814527]

    print(ret)

    dt = 0.001
    tend = T + dt
    val = np.zeros(shape=(int(np.ceil(tend/dt)), 3))
    tvec = np.zeros(shape=(val.shape[0],))
    t = 0
    i = 0
    while t <= tend:
        val[i] = evaluatePolynom(ret, t)
        tvec[i] = t
        t += dt
        i += 1

    fig, axs = plt.subplots(3, 1, sharex=True)
    labels = ["s in m", "v in m/s", "a in m/s2"]
    for i in range(3):
        axs[i].plot(tvec, val[:,i])
        axs[i].set_ylabel(labels[i])
        axs[i].grid()
    plt.show()


if __name__ == "__main__":
    symbolicInverse()
    main()