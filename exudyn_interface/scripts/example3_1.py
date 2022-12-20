#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 20.12.2022

Copy of example3.py but with ros interface outsourced
"""

from anyio import typed_attribute
import exudyn as exu
print("Using Exudyn version ", exu.__version__)
from exudyn.utilities import *

import numpy as np
print("Using Numpy version ", np.__version__)

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf

import sys
import os

f = os.path.dirname(os.path.abspath(__file__))
# go two directories up
for _ in range(2):
    f = f[0:f.rfind("/")]
sys.path.append(f)  # append [...]/exuCobot directory to system path

from exudyn_interface.scripts.RosExInterface import *

invisible = {'show': False, 'drawSize': -1, 'color': [-1]*4}

def main():

    print("impedance: ", impedanceController)

    RosExInit()

    print("Calibrating. Do not touch robot")

    # init exudyn
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    tRes = 0.001    # step size in s
    tEnd = 10000    # simulation time in s

    g = 9.81    # gravity in m/s^2

    l = 0.5     # z-dim of pendulum
    b = 0.02    # x-y-dim of pendulum

    rho = 0  # density of pendulum in kg/m^3    (ideal pendulum)

    massPendulumTip = 6
    rPendulumTip = 0.03      # radius of pendulum tip -> matches red user interaction sphere at robot (d=6cm)

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # parameter and ground with rectangle visualisation

    background = GraphicsDataCheckerBoard(point=[0,-l/2,-b/2], color=[0.7]*3+[1], alternatingColor=[0.8]*3+[1])

    bGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))#,
                                         #visualization=VObjectGround(graphicsData=[background])))

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
                                        position = [0, 0, 0], 
                                        rotationMatrix = np.eye(3), 
                                        angularVelocity = np.zeros(3),
                                        velocity=np.zeros(3),
                                        gravity = [0, 0, 0], 
                                        graphicsDataList = [graphics2a])

    [nTip, bTip]=AddRigidBody(mainSys = mbs, 
                              inertia = inertiaPendulumTip, 
                              nodeType = str(exu.NodeType.RotationEulerParameters), 
                              position = [0, 0, -l], 
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
    d[3,3] = 0.5
    d[4,4] = 0.5
    d[5,5] = 0.5

    mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mGround, mPendulumTip],
                                        stiffness=k,
                                        damping=d,
                                        visualization=invisible))

    # external applied forces
    def UFloadX(mbs, t, load):
        #return -extEfforts[0]   # somehow x in robot coordinate system is opposite to exudyn?
        return 0

    def UFloadY(mbs, t, load):
        return extEfforts[1]
        # return 0

    def UFloadZ(mbs, t, load):
        return extEfforts[2]

    mFx = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nTip, coordinate=0))
    mFy = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nTip, coordinate=1))
    mFz = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nTip, coordinate=2))
    
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

    # initialisation of some variables
    posOffset = np.array([0, 0, 0]) # offset between user-interact position in simulation and robot world space 
    firstPose = True                # flag to determine first step; needed to to calculate posOffset
    T = np.eye(4)                   # for full coordinate transformation

    # publishing each and every step is too much, this slows the connection down
    # thus publish every xth pose, only
    xPublish = 6
    xPublishCounter = 0

    def PreStepUserFunction(mbs, t):
        nonlocal firstPose
        nonlocal posOffset
        nonlocal T
        nonlocal xPublishCounter

        if xPublishCounter == 0:
            # read current kinematic state and orientation
            pos_ = mbs.GetSensorValues(mbs.variables['pos'])
            vel_ = mbs.GetSensorValues(mbs.variables['vel'])
            acc_ = mbs.GetSensorValues(mbs.variables['acc'])

            rot_ = mbs.GetSensorValues(mbs.variables['rotation'])
            
            # convert data to numpy arrays
            pos = np.array(pos_)
            vel = np.array(vel_)
            acc = np.array(acc_)
            rot = np.array(rot_)

            # in first iteration, calculate posOffset and T
            if firstPose:

                gsp = getGlobalStartPos()
                print("calculating offset")
                print("globalStartPos = ", gsp)
                print("exu pos = ", pos)
                posOffset = gsp - pos
                # full coordinate transformation
                posOffset = np.expand_dims(posOffset, axis=1)
                # TODO: don't forget rotation
                #T = np.concatenate((trafoMat, posOffset), axis=1)
                #T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
                firstPose = False

            # initilaize container
            posGlobal = [0, 0, 0]

            # as for now no rotation is required, it is faster to compute coordinates without matrix multiplication
            posGlobal[0] = pos[0] + posOffset[0][0]
            posGlobal[1] = pos[1] + posOffset[1][0]
            posGlobal[2] = pos[2] + posOffset[2][0]

            # calculate angle
            angleX = float(round(180+np.rad2deg(rot[0]), 4))

            #print(angleX, type(angleX))

            publish(posGlobal, vel, acc, angleX, t)

        xPublishCounter += 1

        if xPublishCounter >= xPublish:
            xPublishCounter = 0

        # prestep-userfunction has to return true, else simulation stops
        return True

    mbs.SetPreStepUserFunction(PreStepUserFunction)

    if not calibrate(): # robot - exudyn calibration
        return

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

    viewMatrix = np.eye(3)  @ RotationMatrixZ(np.pi/2)@ RotationMatrixX(np.pi/2)
    SC.visualizationSettings.general.autoFitScene = False
    SC.visualizationSettings.openGL.initialModelRotation = viewMatrix
    SC.visualizationSettings.openGL.initialCenterPoint = [0., -l/2, 0.] # screen coordinates, not model coordinates
    SC.visualizationSettings.openGL.initialZoom = 0.5

    #SC.visualizationSettings.window.renderWindowSize = [1920, 1080]
    SC.visualizationSettings.window.renderWindowSize = [960, 640]
    #SC.visualizationSettings.window.renderWindowSize = [480, 320]

    # exudyn magic
    exu.StartRenderer()
    #mbs.WaitForUserToContinue() # space/q to start - q to end
    exu.SolveDynamic(mbs, simulationSettings)
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


if __name__ == "__main__":
  
    if len(sys.argv) >= 2 and sys.argv[1] == '-i':
        impedanceController = True

    main()
    cleanUp()
