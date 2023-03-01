#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 07.02.2023

Exudyn example with VR enabled
"""

import exudyn as exu
print("Using Exudyn version ", exu.__version__)
from exudyn.utilities import *

import numpy as np

from RobotVrInterface import RobotVrInterface

import sys

def main(client, useImpedanceController):

    ## Simulation parameters
    tRes = 0.001    # step size in s
    tEnd = 10000    # simulation time in s

    g = 9.81    # gravity in m/s^2

    l = 0.5     # z-dim of pendulum
    b = 0.02    # x-y-dim of pendulum

    rho = 0  # density of pendulum in kg/m^3    (ideal pendulum)

    massPendulumTip = 6
    rPendulumTip = 0.03      # radius of pendulum tip -> matches red user interaction sphere at robot (d=6cm)



    # init exudyn
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    robotVrInterface = RobotVrInterface(mbs, client, useImpedanceController)

    viewMatrix = np.eye(3) @ RotationMatrixZ(np.pi) @ RotationMatrixX(np.pi/2)
    robotVrInterface.setRotationMatrix(viewMatrix)

    interactionPointOffset = np.array([0,-l,0])
    origin = robotVrInterface.determineRobotStartPosition(interactionPointOffset)

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    bGround = mbs.AddObject(ObjectGround(referencePosition=origin))

    graphics2a = GraphicsDataOrthoCube(xMin=-b/2, xMax=b/2,
                                       yMin=-b/2, yMax=b/2,
                                       zMin=-l, zMax=0,
                                       color=[0.4, 0.4, 0.4, 1])

    graphics2b = GraphicsDataSphere(point=[0,0,0],
                                    radius=rPendulumTip,
                                    color=(1,0,0,1),
                                    nTiles=64)

    inertiaPendulum = InertiaCuboid(density=rho,
                                    sideLengths=(b, b, l))

    inertiaPendulum.Translated([0,0,-l/2])


    inertiaPendulumTip = InertiaSphere(mass=massPendulumTip,
                                       radius=rPendulumTip)

    [nPendulum, bPendulum]=AddRigidBody(mainSys = mbs, 
                                        inertia = inertiaPendulum, 
                                        nodeType = str(exu.NodeType.RotationEulerParameters), 
                                        position = origin, 
                                        rotationMatrix = np.eye(3), 
                                        angularVelocity = np.zeros(3),
                                        velocity=np.zeros(3),
                                        gravity = [0, 0, 0], 
                                        graphicsDataList = [graphics2a])

    [nTip, bTip]=AddRigidBody(mainSys = mbs, 
                              inertia = inertiaPendulumTip, 
                              nodeType = str(exu.NodeType.RotationEulerParameters), 
                              position = [origin[0], origin[1], origin[2]-l], 
                              rotationMatrix = np.eye(3), 
                              angularVelocity = np.zeros(3),
                              velocity= [0,0,0],
                              gravity = [0, 0, 0], 
                              graphicsDataList = [graphics2b])


    # create markers:
    # mGround lies on the ground where the pendulum is connected
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bGround, localPosition=[0, 0, 0.]))

    # mPendulumTop lies on the upper end of the pendulum where it is connected with ground
    mPendulumTop = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPendulum, localPosition=[0, 0., 0.]))

    # mPendulumMid lies in the middle of the pendulum where the gravitational force applies
    mPendulumMid = mbs.AddMarker(MarkerBodyPosition(bodyNumber=bPendulum, localPosition=[0., 0., -l/2]))

    # mPendulumTip and mTip lie on the lower end of the pendulum where the user can interact
    mPendulumTip = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPendulum, localPosition=[0., 0., -l]))
    mTip = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bTip))

    # a generic joint to allow rotation around x, y and z
    mbs.AddObject(GenericJoint(markerNumbers=[mGround, mPendulumTop],
                            constrainedAxes=[1,1,1,0,0,0],
                            visualization=VObjectJointGeneric(axesRadius=0.004, axesLength=0.1)))
 
    # stick pendulum body to pendulum rod
    mbs.AddObject(GenericJoint(markerNumbers=[mPendulumTip, mTip],
                               constrainedAxes=[1,1,1,1,1,1],
                               visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.01)))   # just very small visualization so you don't see it
    
    # gravitational force
    mbs.AddLoad(Force(markerNumber=mPendulumMid, loadVector=[0, 0, -inertiaPendulum.mass*g]))
    mbs.AddLoad(Force(markerNumber=mPendulumTip, loadVector=[0, 0, -inertiaPendulumTip.mass*g]))

    # friction
    k = np.zeros(shape=(6,6))
    d = np.zeros(shape=(6,6))
    #d[0,0] = 1 # constrained directions
    #d[1,1] = 1
    #d[2,2] = 1
    d[3,3] = 1#0.5
    d[4,4] = 1#0.5
    d[5,5] = 1#0.5

    mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mGround, mPendulumTip],
                                        stiffness=k,
                                        damping=d,
                                        visualization={'show': False, 'drawSize': -1, 'color': [-1]*4}))

    mbs = robotVrInterface.setHand(mbs)

    if client == 1:

        # external applied forces
        def UFloadX(mbs, t, load):
            #return robotVrInterface.getExternalEfforts()[0]
            return 0

        def UFloadY(mbs, t, load):
            return robotVrInterface.getExternalEfforts()[1]
            # return 0

        def UFloadZ(mbs, t, load):
            return robotVrInterface.getExternalEfforts()[2]

        mFx = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nTip, coordinate=0))
        mFy = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nTip, coordinate=1))
        mFz = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nTip, coordinate=2))
        
        # TODO: das sollte auch als Vektor gehen

        mbs.AddLoad(LoadCoordinate(markerNumber=mFx,
                                loadUserFunction=UFloadX))
        mbs.AddLoad(LoadCoordinate(markerNumber=mFy,
                                loadUserFunction=UFloadY))
        mbs.AddLoad(LoadCoordinate(markerNumber=mFz,
                                loadUserFunction=UFloadZ))

    # sensor for position of endpoint of pendulum
    sensorPos = mbs.AddSensor(SensorBody(bodyNumber=bTip,
                                         outputVariableType=exu.OutputVariableType.Position))
    sensorVel = mbs.AddSensor(SensorBody(bodyNumber=bTip,
                                         outputVariableType=exu.OutputVariableType.Velocity))
    sensorAcc = mbs.AddSensor(SensorBody(bodyNumber=bTip,
                                         outputVariableType=exu.OutputVariableType.Acceleration))

	# store sensor value of each step in mbs variable, so that is accessible from user function
    mbs.variables['pos'] = sensorPos
    mbs.variables['vel'] = sensorVel
    mbs.variables['acc'] = sensorAcc

    # sensor for rotation (orientation) of endpoint of pendulum
    sensorRot = mbs.AddSensor(SensorBody(bodyNumber=bTip,
                                         outputVariableType=exu.OutputVariableType.Rotation))

    # store sensor value of each step in mbs variable, so that is accessible from user function
    mbs.variables['rotation'] = sensorRot


    def PreStepUserFunction(mbs, t):

        #try:
        #    print(SC.GetRenderState()['openVR']['controllerPoses'])
        #except:
        #    pass
        mbs = robotVrInterface.update(mbs, SC, t)
        
        return True

    mbs.SetPreStepUserFunction(PreStepUserFunction)

    # assemble multi body system with all previous specified properties and components
    mbs.Assemble()

    # set simulation settings
    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/tRes)
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10
    simulationSettings.timeIntegration.verboseMode = 1 # if 0 no output; higher --> more output information about solver
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
    simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
    
    simulationSettings.timeIntegration.simulateInRealtime = True    # crucial for operating with robot
    simulationSettings.displayStatistics = True
    simulationSettings.solutionSettings.solutionInformation = "3D Pendulum"
    simulationSettings.solutionSettings.writeSolutionToFile = False
    
    SC.visualizationSettings.general.autoFitScene = False
    SC.visualizationSettings.openGL.initialModelRotation = viewMatrix
    SC.visualizationSettings.openGL.initialCenterPoint = [0., -l/2, 0.] # screen coordinates, not model coordinates
    SC.visualizationSettings.openGL.initialZoom = 0.5
    
    robotVrInterface.setSettings(SC)    # TODO: is everything above included?

    # exudyn magic
    exu.StartRenderer()
    
    #mbs.WaitForUserToContinue() # space/q to start - q to end
    
    exu.SolveDynamic(mbs, simulationSettings)
    # TODO nur Daten aus solutionViewer auslesen und viewer aktualisieren? oder im solutionViewer aktualisier? auf jeden Fall nicht simulieren
    
    #SC.WaitForRenderEngineStopFlag()
    
    exu.StopRenderer() #safely close rendering window!


if __name__ == "__main__":

    if False:
        # for debugging
        main(2, False)

    else:
        from RobotVrInterface import handleArgv
        client, useImpedance = handleArgv(sys.argv)
        main(client, useImpedance)

