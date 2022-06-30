#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 20.06.2022

Example-script for testing interface between exudyn and robot-controller
Implements single pendulum moving in yz-plane
"""

import exudyn as exu
print("Using Exudyn version ", exu.__version__)
from exudyn.itemInterface import *
from exudyn.utilities import *

import numpy as np
print("Using Numpy version ", np.__version__)

from spatialmath import SE3


def main():
    # erstelle Systemcontainer und
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    tRes = 0.01 # step size
    tEnd = 3

    g = 9.81    # gravity

    # start position of robot in robot frame in meters
    globalStartPos = np.array([0.3, 0, 0.2])

    # align mbs with robot axis
    rotMat = np.array([[0, 0, 1],
                       [0, 1, 0],
                       [1, 0, 0]])

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # parameter und ground mit Rechteck Visualisierung
    #rigid pendulum:
    rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
    background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
    oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
    a = 0.5     #x-dim of pendulum
    b = 0.05    #y-dim of pendulum
    massRigid = 12
    inertiaRigid = massRigid/12*(2*a)**2

    # Körper bestehend aus Rigid2D Node (Koordinaten) und Objekt RigidBody2D (physikal. Eigenschaften und Visualisierung)
    graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[-1,0.5,-np.pi/2], initialVelocities=[0,0,5]));
    oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

    # Erstellen der Marker:
    # mR1 ist das obere Ende des körpers und der Punkt an dem das Gelenk angebracht wird
    # mR2 ist im Mittelpunkt des Körpers
    # mR3 ist am unteren Ende des Pendelkörpers
    # mG0 ist starr mi dem Ground verbunden
    # durch ein RevoluteJoint2D werden die Marker mG0 und und mR1 verbunden -> nur Verdrehung zueinander möglich
    mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5,0.,0.])) #support point
    mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) #mid point
    mR3 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.5,0.,0.])) #end point
    mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-1,1.,0.]))
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))

    # Erstelle Gravitationskraft auf den Körper
    mbs.AddLoad(Force(markerNumber=mR2, loadVector=[0, -massRigid*g, 0]))   # Gravity

    # Positionssensor für Pendel-Endpunkt
    sensorPos = mbs.AddSensor(SensorMarker(markerNumber=mR3,
                                           outputVariableType=exu.OutputVariableType.Position))
    mbs.variables['pos'] = sensorPos

    posOffset = np.array([0, 0, 0])
    i = 0
    T = np.eye(4)    # full coordinate transformation

    def PreStepUserFunction(mbs, t):
        nonlocal i
        nonlocal posOffset
        nonlocal T

        pos_ = mbs.GetSensorValues(mbs.variables['pos'])
        pos = np.array(pos_)

        if i == 0:
            posOffset = globalStartPos - pos
            # full coordinate transformation
            posOffset = np.expand_dims(posOffset, axis=1)
            #T = np.concatenate((rotMat, posOffset), axis=1)
            #T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)


        posFinal = [0, 0, 0]

        #pos = np.array([pos[0], pos[1], pos[2], 1])
        #posFinal = T @ pos
        
        posFinal[0] = pos[2] + posOffset[2][0]
        posFinal[1] = pos[1] + posOffset[1][0]
        posFinal[2] = pos[0] + posOffset[0][0]

        print(i, posFinal, t)
        i += 1

        return True

    mbs.SetPreStepUserFunction(PreStepUserFunction)

    # Zuweisen der Anfangswerte, verbinden der Komponenten, Checks
    mbs.Assemble()

    # Setzen der Simulationseinstellungen: Simulationsdauer, Toleranzen, Löser, Informationen über Lösung
    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/tRes)  # Anzahl Schritte
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
    simulationSettings.timeIntegration.verboseMode = 1 # wenn 0 kein Output; höher --> mehr Output informationen über Solver
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 # Spektralradius des Lösers --> Siehe L
    simulationSettings.timeIntegration.simulateInRealtime = True    # crucial for operating with robot
    simulationSettings.displayStatistics = True
    exu.showHints = True


    simulationSettings.solutionSettings.solutionInformation = "2D Pendel"

    # Starte Renderer
    exu.StartRenderer()

    # warte auf Userinput (Space)
    mbs.WaitForUserToContinue() # space/q to start - q to end

    # Löse das System
    exu.SolveDynamic(mbs, simulationSettings)

    # schließe Visualisierung nach dem StopFlag (q)
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

    #print(mbs.variables['sensorRecord'+str(sensorPos)])
    import os
    os.remove("coordinatesSolution.txt")


if __name__ == "__main__":
    main()