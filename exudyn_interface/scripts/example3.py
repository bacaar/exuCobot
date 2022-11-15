#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 08.10.2022

Example-script for testing interface between exudyn and robot-controller
Implements 3d pendulum
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

invisible = {'show': False, 'drawSize': -1, 'color': [-1]*4}

f = os.path.dirname(os.path.abspath(__file__))
# go two directories up
for _ in range(2):
    f = f[0:f.rfind("/")]
sys.path.append(f)  # append [...]/exuCobot directory to system path

from util.scripts.util.common import createPoseStampedMsg

from util.msg import segmentCommand

logFile = None
impedanceController = False  # default

# force calibration
effortsCalibrated = False
nDesiredCalibrationValues = 100    # external efforts are sent with 30Hz from the robot, so calibration progress takes nCalibrationValues/30 seconds to finish
calibrationValues = np.zeros(shape=(nDesiredCalibrationValues, 6))
nGotCalibrationValues = 0   # iteration variable

# global variable for external forces and moments (combined => efforts)
extEfforts = np.zeros(shape=(6, 1))

# measured force and torque by robot are not equal zero at rest, so store first measured force and torque (assumption: at rest) and substract them of every other one measured
effortsOffset = np.zeros(shape=(6, 1))
effortsThreshold = np.zeros(shape=(6, 1))
effortsThresholdFactor = 10  # factor for scaling threshold

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
        # TODO Logger
        #print("Global start pos set to ", globalStartPos)


def externalEffortCallback(data):
    """
    callback function for external forces and torques, applied by the user on the robot
    for simplicity reasons, forces and torques are denoted as efforts here

    every time new external efforts are registered by the robot(which happens continuously), 
    they are published and received by this callback function. Here they are written into 
    a global variable, in order to be processed by exudyn

    :param geometry_msgs.msg.WrenchStamped data: received message
    """

    global effortsCalibrated
    global effortsOffset
    global effortsThreshold
    global nGotCalibrationValues

    # before operation, calibrate forces
    if not effortsCalibrated:
        if nGotCalibrationValues < nDesiredCalibrationValues:

            calibrationValues[nGotCalibrationValues] = np.array([data.wrench.force.x,
                                                                 data.wrench.force.y,
                                                                 data.wrench.force.z,
                                                                 data.wrench.torque.x,
                                                                 data.wrench.torque.y,
                                                                 data.wrench.torque.z])
            nGotCalibrationValues += 1
        else:
            for i in range(6):
                
                # use mean value as offset
                effortsOffset[i] = np.mean(calibrationValues[:,i])

                # use peak to peak value as threshold
                effortsThreshold[i] = np.abs(np.max(calibrationValues[:,i]) - np.min(calibrationValues[:,i]))*effortsThresholdFactor

            effortsCalibrated = True

            #for i in range(6):
            #    print("effort offset: ", effortsOffset[i])
            #    print("effort threshold: ", effortsThreshold[i])

            # TODO Logger
            # print("Effort calibration done")

    # operating mode
    else:
        
        efforts = np.zeros(shape=(6,))

        # get forces and substract offset
        efforts[0] = data.wrench.force.x - effortsOffset[0]
        efforts[1] = data.wrench.force.y - effortsOffset[1]
        efforts[2] = data.wrench.force.z - effortsOffset[2]

        # get torques and substract offset
        efforts[3] = data.wrench.torque.x - effortsOffset[3]
        efforts[4] = data.wrench.torque.y - effortsOffset[4]
        efforts[5] = data.wrench.torque.z - effortsOffset[5]

        # additional to force offset, there is also some noise on force/torque measurement

        effortNames = ["fx", "fy", "fz", "mx", "my", "mz"]

        for i in range(6):
            if abs(efforts[i]) > effortsThreshold[i]:
                extEfforts[i] = efforts[i]
                #print("effort", effortNames[i], "=", extEfforts[i])
            else:
                extEfforts[i] = 0


def main():

    print("impedance: ", impedanceController)

    # init ros
    rospy.init_node('ExudynExample3', anonymous=True)

    # publisher for pendulum poses (=endeffector positions)
    
    if impedanceController:
        pub = rospy.Publisher('/my_cartesian_impedance_controller/setTargetPose', PoseStamped, queue_size=1000)
        pubF = rospy.Publisher('/my_cartesian_impedance_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)
    else:
        pub = rospy.Publisher('/my_cartesian_velocity_controller/setTargetPose', segmentCommand, queue_size=1000)
        pubF = rospy.Publisher('/my_cartesian_velocity_controller/analysis/getExternalForce', WrenchStamped, queue_size=10)

    # subscriber for external forces
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalEffortCallback)

    # subscriber for current pose
    if impedanceController:
        globalStartPosSub = rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, currentPoseCallback)
    else:
        globalStartPosSub = rospy.Subscriber("/my_cartesian_velocity_controller/getCurrentPose", PoseStamped, currentPoseCallback)

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
                                    sideLengths=(l, b, b))

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
                              velocity=np.zeros(3),
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
    d[3,3] = 1
    d[4,4] = 1
    d[5,5] = 1

    mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mGround, mPendulumTop],
                                        stiffness=k,
                                        damping=d,
                                        visualization=invisible))

    # external applied forces
    def UFloadX(mbs, t, load):
        #return -extEfforts[0]   # somehow x in robot coordinate system is opposite to exudyn?
        return 0

    def UFloadY(mbs, t, load):
        return extEfforts[1]

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

    global logFile
    if impedanceController:
        logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynIC.csv", "w")
    else:
        logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudynVC.csv", "w")
    logFile.write("rt,dt,px,py,pz,vx,vy,vz,ax,ay,az\n")

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
                posOffset = globalStartPos - pos
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

                msg.x.vel = vel[0]
                msg.y.vel = vel[1]
                msg.z.vel = vel[2]

                msg.x.acc = acc[0]
                msg.y.acc = acc[1]
                msg.z.acc = acc[2]

                msg.dt = segment_duration

            pub.publish(msg)

            logFile.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(tsend.to_sec(), segment_duration, posGlobal[0], posGlobal[1], posGlobal[2], vel[0], vel[1], vel[2], acc[0], acc[1], acc[2]))

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

    while not globalStartPosSet:
        if rospy.Time.now() - t0 > rospy.Duration(5):
            print("ERROR: Did not get any robot position after 5 seconds of waiting")
            return

    globalStartPosSub.unregister()  # we don't need current pose anymore

    # wait for calibration to finish
    print("Waiting for effort calibration")
    while not effortsCalibrated:
        pass

    print("Everything ready to go, start using robot now")

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
    logFile.close()
