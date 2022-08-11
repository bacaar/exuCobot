#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 28.07.2022

Example-script for testing interface between exudyn and robot-controller
Implements 3D pendulum
"""

import exudyn as exu
print("Using Exudyn version ", exu.__version__)
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import RotationMatrixX, RotationMatrixZ

import numpy as np
print("Using Numpy version ", np.__version__)

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf

import sys
import os


f = os.path.dirname(os.path.abspath(__file__))
# go to directories up
for _ in range(2):
    f = f[0:f.rfind("/")]
print(f)
sys.path.append(f)  # append [...]/exuCobot directory to system path

from util.scripts.util.common import createPoseStampedMsg


# global variable for external forces and moments (combined => efforts)
extEfforts = np.zeros(shape=(6, 1))


def externalForceCallback(data):
    """
    callback function for external forces and moments, applied by the user on the robot

    every time new external efforts are registered by the robot(which happens continuously), 
    they are published and received by this callback function. Here they are written into 
    another variable, in order to be processed by exudyn

    :param geometry_msgs.msg.WrenchStamped data: received message
    """

    #print("Got efforts " + str(data.header.seq))

    # get forces
    fx = data.wrench.force.x
    fy = data.wrench.force.y
    fz = data.wrench.force.z

    # get moments
    mx = data.wrench.torque.x
    my = data.wrench.torque.y
    mz = data.wrench.torque.z

    # save for later use
    global extEfforts

    # sensor is not that precise, so everything below 5N has to be neglected because of sensornoise
    # this threshold has been set empirically. For other configurations it might be different. TODO
    threshold = 4
    if abs(fy) >= threshold:
        extEfforts[0] = fy
        print("fy = " + str(fy))
    else:
        extEfforts[0] = 0
    if abs(fz) >= threshold:
        extEfforts[1] = fz
        print("fz = " + str(fz))
    else:
        extEfforts[1] = 0
    if abs(fx) >= threshold:
        extEfforts[2] = fx
        print("fx = " + str(fx))
    else:
        extEfforts[2] = 0
    extEfforts[3] = my
    extEfforts[4] = mz
    extEfforts[5] = mx


def main():
    
    # init ros
    rospy.init_node('ExudynExample2', anonymous=True)

    # publisher for pendulum poses (=endeffector positions)
    pub = rospy.Publisher('/my_cartesian_impedance_controller/setDesiredPose', PoseStamped, queue_size=1000)
    pubF = rospy.Publisher('/my_cartesian_impedance_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)

    # subscriber for external forces
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalForceCallback)
    
    # init exudyn
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    # simulation parameters
    tRes = 0.001    # step size in s
    tEnd = 1000     # simulation time in s

    g = 9.81    # gravity in m/s^2

    # pendulum parameters
    a = 0.5     # length of pendulum in m
    b = 0.05    # width of pendulum in m
    rho = 2700  # density of pendulum in kg/m^3

    # start position of robot in robot base frame in meters
    globalStartPos = np.array([0.4, 0, 0.2])

    # create ground
    oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))

    inertiaPendulum = InertiaCuboid(density=rho,
                                    sideLengths=(a, b, b))

    graphicsBodyPendulum = GraphicsDataOrthoCube(xMin=-b/2, xMax=b/2,
                                                 yMin=-b/2, yMax=b/2,
                                                 zMin=0, zMax=a,
                                                 color=[1, 0, 0, 0.5])

    [nPendulum, bPendulum]=AddRigidBody(mainSys = mbs, 
                                        inertia = inertiaPendulum, 
                                        nodeType = str(exu.NodeType.RotationEulerParameters), 
                                        position = [0, 0, -a], 
                                        rotationMatrix = np.eye(3), 
                                        angularVelocity = np.zeros(3),
                                        velocity=np.zeros(3),
                                        gravity = [0, 0, 0], 
                                        graphicsDataList = [graphicsBodyPendulum])

    # connect pendulum with ground
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround,
                                            localPosition = [0, 0, 0]))
    mP0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPendulum,
                                        localPosition = [0, 0, a]))
    mbs.AddObject(GenericJoint(markerNumbers=[mGround, mP0],
                               constrainedAxes=[1,1,1,0,0,1],
                               visualization=VObjectJointGeneric(axesRadius=0.004, axesLength=0.1)))

    # add gravity
    mP1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPendulum,
                                        localPosition = [0, 0, a/2]))
    mbs.AddLoad(Force(markerNumber=mP1, loadVector=[0, 0, -inertiaPendulum.mass*g]))

    # external applied forces
    def UFloadX(mbs, t, load):
        return extEfforts[0]
        #return 2
        #return 0

    def UFloadY(mbs, t, load):
        return extEfforts[1]
        #if t < 1:
        #    return 10
        #else: 
        #    return 0

    def UFloadZ(mbs, t, load):
        return extEfforts[2]
        #return 2
        #return 0

    mFx = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nPendulum, coordinate=0))   # TODO: wo am Körper???
    mFy = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nPendulum, coordinate=1))
    mFz = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nPendulum, coordinate=2))
    
    mbs.AddLoad(LoadCoordinate(markerNumber=mFx,
                               loadUserFunction=UFloadX))
    mbs.AddLoad(LoadCoordinate(markerNumber=mFy,
                               loadUserFunction=UFloadY))
    mbs.AddLoad(LoadCoordinate(markerNumber=mFz,
                               loadUserFunction=UFloadZ))

    # add friction
    k = np.zeros(shape=(6,6))
    d = np.zeros(shape=(6,6))
    d[3,3] = 10
    d[4,4] = 10

    mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mGround, mP0],
                                        stiffness=k,
                                        damping=d))

    # add marker at bottom of pendulum for sensor
    mPEnd = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPendulum,
                                          localPosition = [0, 0, 0]))

    # sensor for position of endpoint of pendulum
    sensorPos = mbs.AddSensor(SensorMarker(markerNumber=mPEnd,
                                           outputVariableType=exu.OutputVariableType.Position))
    # store sensor value of each step in mbs variable, so that is accessible from user function
    mbs.variables['pos'] = sensorPos

    # sensor for rotation (orientation) of endpoint of pendulum
    sensorRot = mbs.AddSensor(SensorBody(bodyNumber=bPendulum,
                                         outputVariableType=exu.OutputVariableType.Rotation))
    # store sensor value of each step in mbs variable, so that is accessible from user function
    mbs.variables['rotation'] = sensorRot

    # initialisation of some variables
    posOffset = np.array([0, 0, 0]) # offset between user-interact position in simulation and robot world space 
    firstPose = True                # flag to determine first step; needed to to calculate posOffset
    T = np.eye(4)                   # for full coordinate transformation

    # publishing each and every step is too much, this slows the connection down
    # thus publish every xth pose, only
    xPublish = 10
    xPublishCounter = 0

    def PreStepUserFunction(mbs, t):
        nonlocal firstPose
        nonlocal posOffset
        nonlocal T
        nonlocal xPublishCounter
        
        if xPublishCounter == 0:
            # read current position and orientation
            pos_ = mbs.GetSensorValues(mbs.variables['pos'])
            rot_ = mbs.GetSensorValues(mbs.variables['rotation'])
            pos = np.array(pos_)
            rot = np.array(rot_)

            # in first iteration, calculate posOffset and T
            if firstPose:
                buf = [pos[0], pos[1], pos[2]]
                posOffset = globalStartPos - buf
                # full coordinate transformation
                posOffset = np.expand_dims(posOffset, axis=1)
                # TODO: don't forget rotation
                #T = np.concatenate((trafoMat, posOffset), axis=1)
                #T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
                firstPose = False

            # initilaize container
            posGlobal = [0, 0, 0]

            # transform simulation coordinates to robot world coordinates
            # TODO: rotation
            #pos = np.array([pos[0], pos[1], pos[2], 1])
            #posGlobal = T @ pos
            #posGlobal = posGlobal[:3]

            # as for now no rotation is required, it is faster to compute coordinates without matrix multiplication
            posGlobal[0] = pos[0] + posOffset[0][0]
            posGlobal[1] = pos[1] + posOffset[1][0]
            posGlobal[2] = pos[2] + posOffset[2][0]

            # calculate angle
            angle = np.array([180.0, 180.0, 180.0])
            angle += np.round(np.rad2deg(rot), 4)

            #print(angleX, type(angleX))

            # compose message and publish
            tsend = rospy.Time.now()
            msg = createPoseStampedMsg(posGlobal, angle, tsend)
            pub.publish(msg)

            # publish current external force
            msgF = WrenchStamped()
            msgF.header.stamp = tsend
            msgF.wrench.force.x = extEfforts[0]
            msgF.wrench.force.y = extEfforts[1]
            msgF.wrench.force.z = extEfforts[2]
            pubF.publish(msgF)

        xPublishCounter += 1

        if xPublishCounter >= xPublish:
            xPublishCounter = 0

        # prestep-userfunction has to return true, else simulation stops
        return True

    mbs.SetPreStepUserFunction(PreStepUserFunction)                           

    # assemble multi body system with all previous specified properties and components
    mbs.Assemble()

    # go to start position
    try:
        # TODO: why are following two lines not working?
        # from util.goToPose import goToPose
        # goToPose(globalStartPos[0], globalStartPos[1], globalStartPos[2], 180, 0, 0)
        from util.scripts.util.goToPose import goToPose

    except ModuleNotFoundError:
        print("Could not go to starting position", globalStartPos)
        userInput = input("Is robot already there? (y/n): ")
        if userInput != "y":
            exit(-1)

    # go to global starting postion
    print("Moving to starting pose...")
    #goToPose(globalStartPos[0], globalStartPos[1], globalStartPos[2], 180, 0, 0, True)

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
    simulationSettings.timeIntegration.simulateInRealtime = True    # crucial for operating with robot
    simulationSettings.displayStatistics = True
    simulationSettings.solutionSettings.solutionInformation = "2D Pendulum"

    viewMatrix = np.eye(3)  @ RotationMatrixZ(np.pi/2)@ RotationMatrixX(np.pi/2)
    SC.visualizationSettings.general.autoFitScene = False
    SC.visualizationSettings.openGL.initialModelRotation = viewMatrix

    # exudyn magic
    exu.StartRenderer()
    mbs.WaitForUserToContinue() # space/q to start - q to end
    exu.SolveDynamic(mbs, simulationSettings)
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

    # remove "coordinatesSolution.txt" as it isn't needed
    os.remove("coordinatesSolution.txt")


if __name__ == "__main__":
    main()