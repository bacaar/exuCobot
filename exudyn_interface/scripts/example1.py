#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 20.06.2022

Example-script for testing interface between exudyn and robot-controller
Implements single pendulum moving in yz-plane (of robot base coordinate system)
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

from util.scripts.util.common import createPoseStampedMsg

from util.msg import segmentCommand

logFile = None

# global variable for external forces and moments (combined => efforts)
extEfforts = np.zeros(shape=(6, 1))

# start position of robot in robot base frame in meters
globalStartPos = np.array([0.4, 0, 0.2])    # default, but not precise enough
globalStartPosSet = False
def currentPoseCallback(data):
    """
    callback function to set global robot position at beginning of program
    -> globalStartPos
    """

    global globalStartPosSet
    global globalStartPos
    if not globalStartPosSet:
        globalStartPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        globalStartPosSet = True
        


def externalForceCallback(data):
    """
    callback function for external forces and moments, applied by the user on the robot

    every time new external efforts are registered by the robot(which happens continuously), 
    they are published and received by this callback function. Here they are written into 
    another variable, in order to be processed by exudyn

    :param geometry_msgs.msg.WrenchStamped data: received message
    """

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


def main(impedanceController = False):

    # flag for activating testing mode
    # if True, friction is disabled, robot doesn't go to starting pose and simulation starts with initial velocity
    theoreticalTest = False

    # init ros
    rospy.init_node('ExudynExample1', anonymous=True)

    # publisher for pendulum poses (=endeffector positions)
    
    if impedanceController:
        pub = rospy.Publisher('/my_cartesian_impedance_controller/setTargetPose', PoseStamped, queue_size=1000)
        pubF = rospy.Publisher('/my_cartesian_impedance_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)
    else:
        pub = rospy.Publisher('/my_cartesian_velocity_controller/setTargetPose', segmentCommand, queue_size=1000)
        pubF = rospy.Publisher('/my_cartesian_velocity_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)

    # subscriber for external forces
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalForceCallback)

    # subscriber for current pose
    if impedanceController:
        globalStartPosSub = rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)
    else:
        globalStartPosSub = rospy.Subscriber("/my_cartesian_velocity_controller/getCurrentPose", PoseStamped, currentPoseCallback)
        

    # init exudyn
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    tRes = 0.001    # step size in s
    tEnd = 10000     # simulation time in s

    g = 9.81    # gravity in m/s^2

    a = 0.5     # x-dim of pendulum
    b = 0.05    # y-dim of pendulum

    # matrix to transform simulation coordinates to robot world coordinates
    #trafoMat = np.array([[0, 0, 1],
    #                     [1, 0, 0],
    #                     [0, 1, 0]])

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # parameter and ground with rectangle visualisation

    background = GraphicsDataCheckerBoard(point=[0,-a/2,-b/2], color=[0.7]*3+[1], alternatingColor=[0.8]*3+[1])

    oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0],
                                         visualization=VObjectGround(graphicsData=[background])))
    
    if impedanceController:
        massRigid = 6
    else:
        massRigid = 6
    inertiaRigid = massRigid/3 * a**2

    graphics2a = GraphicsDataOrthoCube(xMin=-b/2, xMax=b/2,
                                       yMin=-a,   yMax=0,
                                       zMin=-b/2, zMax=b/2,
                                       color=[0.4, 0.4, 0.4, 1])

    graphics2b = GraphicsDataSphere(point=[0,0,0],
                                    radius=b/sqrt2,
                                    color=(1,0,0,1),
                                    nTiles=64)

    nRigid = None
    if not theoreticalTest:
        nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[0, 0, 0],
                                    initialVelocities=[0, 0, 0]))
    else:
        nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[0, 0, 0],
                                    initialVelocities=[0, 0, 1]))

    nRigid2 = mbs.AddNode(Rigid2D(referenceCoordinates=[0, -a, 0]))
    
    oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid*0.9,
                                       physicsInertia=inertiaRigid,
                                       nodeNumber=nRigid,
                                       visualization=VObjectRigidBody2D(graphicsData=[graphics2a])))

    oRigid2 = mbs.AddObject(RigidBody2D(physicsMass=massRigid*0.1,
                                       physicsInertia=inertiaRigid*0.1,
                                       nodeNumber=nRigid2,
                                       visualization=VObjectRigidBody2D(graphicsData=[graphics2b])))


    # create markers:
    # mG0 lies on the ground where the pendulum is connected
    mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0, 0, 0.]))
    # different marker (type) at same position for friction
    mGF = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0.]))

    # mR1 lies on the upper end of the pendulum where it is connected with ground
    mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0, 0., 0.]))
    # same as above: different type at same position for friction
    mR1F = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid, localPosition=[0, 0., 0.]))

    # mR2 lies in the middle of the pendulum where the gravitational force applies
    mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0., -a/2, 0.]))

    # mR3 lies on the lower end of the pendulum where the user can interact
    mR3 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0., -a, 0.]))

    # a RevoluteJoint2D allows only rotation between markers mG0 and mR1
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0, mR1], visualization=VObjectJointRevolute2D(True,drawSize=b)))
    
    mTip = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2)) 
    # mbs.AddObject(DistanceConstraint(markerNumbers=[mR3, mTip]))
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR3, mTip], visualization=VObjectJointRevolute2D(True,drawSize=b)))
    # gravitational force for pendulum
    mbs.AddLoad(Force(markerNumber=mR2, loadVector=[0, -massRigid*g, 0]))

    # friction
    if not theoreticalTest:
        if impedanceController:
            mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mGF, mR1F],
                                                stiffness=0,
                                                damping=2))
        else:
            mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mGF, mR1F],
                                                stiffness=0,
                                                damping=2))

    # external applied forces
    def UFloadX(mbs, t, load):
        return extEfforts[0]

    mFx = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid2, coordinate=0))
    mFy = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid2, coordinate=1))
    mFz = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid2, coordinate=2))
    mbs.AddLoad(LoadCoordinate(markerNumber=mFx,
                               loadUserFunction=UFloadX))

    # sensor for position of endpoint of pendulum
    #sensorPos = mbs.AddSensor(SensorMarker(markerNumber=mR3,
    #                                       outputVariableType=exu.OutputVariableType.Position))
    sensorPos = mbs.AddSensor(SensorBody(bodyNumber=oRigid2,
                                         outputVariableType=exu.OutputVariableType.Position))
    sensorVel = mbs.AddSensor(SensorBody(bodyNumber=oRigid2,
                                         outputVariableType=exu.OutputVariableType.Velocity))
    sensorAcc = mbs.AddSensor(SensorBody(bodyNumber=oRigid2,
                                         outputVariableType=exu.OutputVariableType.Acceleration))

    # store sensor value of each step in mbs variable, so that is accessible from user function
    mbs.variables['pos'] = sensorPos
    mbs.variables['vel'] = sensorVel
    mbs.variables['acc'] = sensorAcc

    # sensor for rotation (orientation) of endpoint of pendulum
    sensorRot = mbs.AddSensor(SensorBody(bodyNumber=oRigid,
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

    global logFile
    if impedanceController:
        logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynIC.csv", "w")
    else:
        logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynVC.csv", "w")
    logFile.write("rt,dt,px,py,pz,vx,vy,vz,ax,ay,az\n")
    #logFile.write("dt,globalX,globalY,globalZ,dT,dx,dy,dz\n")

    lastGlobalX = 0 # initialization only important for first step, so don't bother
    lastGlobalY = 0
    lastGlobalZ = 0
    lastT = 0

    lastStepTime = 0    # last step time (exudyn time)

    def PreStepUserFunction(mbs, t):
        nonlocal firstPose
        nonlocal posOffset
        nonlocal T
        nonlocal xPublishCounter

        nonlocal lastGlobalX
        nonlocal lastGlobalY
        nonlocal lastGlobalZ
        nonlocal lastT
        nonlocal lastStepTime

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
                buf = [pos[2], pos[0], pos[1]]
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
            posGlobal[0] = pos[2] + posOffset[0][0]
            posGlobal[1] = pos[0] + posOffset[1][0]
            posGlobal[2] = pos[1] + posOffset[2][0]

            # calculate angle
            angleX = float(round(180+np.rad2deg(rot), 4))

            #print(angleX, type(angleX))

            # compose message and publish
            tsend = rospy.Time.now()
            segment_duration = t - lastStepTime     # time from last position to this position
            #print(segment_duration.to_sec(), "\t", t - lastStepTime)
            lastStepTime = t

            if impedanceController:
                msg = createPoseStampedMsg(posGlobal, (angleX, 0, 0), tsend)
            else:
                msg = segmentCommand()
                msg.x.pos = posGlobal[0]
                msg.y.pos = posGlobal[1]
                msg.z.pos = posGlobal[2]

                msg.x.vel = vel[2]
                msg.y.vel = vel[0]
                msg.z.vel = vel[1]

                msg.x.acc = acc[2]
                msg.y.acc = acc[0]
                msg.z.acc = acc[1]

                msg.dt = segment_duration

            if not theoreticalTest:
                pub.publish(msg)

            #dx = posGlobal[0] - lastGlobalX
            #dy = posGlobal[0] - lastGlobalY
            #dz = posGlobal[0] - lastGlobalZ
            #dt = tsendS - lastT

            logFile.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(tsend.to_sec(), segment_duration, posGlobal[0], posGlobal[1], posGlobal[2], vel[2], vel[0], vel[1], acc[2], acc[0], acc[1]))
            #logFile.write("{},{},{},{},\t{},{},{},{}\n".format(tsendS, posGlobal[0], posGlobal[1], posGlobal[2], dt, dx, dy, dz))

            #lastGlobalX = posGlobal[0]
            #lastGlobalY = posGlobal[1]
            #lastGlobalZ = posGlobal[2]
            #lastT = tsendS

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

    # wait for global start position
    print("Waiting for global start position")
    t0 = rospy.Time.now()

    if not theoreticalTest:
        while not globalStartPosSet:
            if rospy.Time.now() - t0 > rospy.Duration(5):
                print("ERROR: Did not get any robot position after 5 seconds of waiting")
                return

    globalStartPosSub.unregister()  # we don't need current pose anymore

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
    simulationSettings.solutionSettings.solutionInformation = "2D Pendulum"
    simulationSettings.solutionSettings.writeSolutionToFile = False

    #SC.visualizationSettings.window.renderWindowSize = [1920, 1080]
    SC.visualizationSettings.window.renderWindowSize = [480, 320]

    # exudyn magic
    exu.StartRenderer()
    #mbs.WaitForUserToContinue() # space/q to start - q to end
    exu.SolveDynamic(mbs, simulationSettings)
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


if __name__ == "__main__":

    impedanceController = False  # default
  
    if len(sys.argv) >= 2 and sys.argv[1] == '-i':
        impedanceController = True

    main(impedanceController)
    logFile.close()
