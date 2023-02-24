#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 30.09.2022

Script for calculating coefficients for one single segment with given start and end state
"""

from PolynomialTrajectoryAnalysis import calcCoefs, evaluatePolynom

# format: t,cpx,cvx,cax,cpy,cvy,cay,cpz,cvz,caz,npx,nvx,nax,npy,nvy,nay,npz,nvz,naz,dt
#para = [6.481000000,0.628468,1.43655e-06,0.000510138,0.0289422,-0.28378,-0.541813,0.196431,0.0134327,0.0789644,0.628468,1.2693e-06,-1.67244e-05,0.0260814,-0.288605,-0.482466,0.19657,0.0144242,0.0991541,0.01]
para = [6.492000000,0.628468,2.77892e-06,0.00176245,0.0257931,-0.289319,-0.187783,0.196584,0.0145703,0.0390477,0.628468,1.33439e-06,-0.000144453,0.0231701,-0.279372,0.994782,0.19672,0.0148201,0.024981,0.01]

polyOder = 5

coefsX = calcCoefs(para[1],para[2],para[3],para[10],para[11],para[12],polyOrder=polyOder, T=para[19])
coefsY = calcCoefs(para[4],para[5],para[6],para[13],para[14],para[15],polyOrder=polyOder, T=para[19])
coefsZ = calcCoefs(para[7],para[8],para[9],para[16],para[17],para[18],polyOrder=polyOder, T=para[19])

print(coefsX)
print(coefsY)
print(coefsZ)