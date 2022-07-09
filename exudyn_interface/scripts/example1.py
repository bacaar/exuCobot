#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 20.06.2022

Example-script for testing interface between exudyn and robot-controller
Implements single pendulum moving in yz-plane (of robot base coordinate system)
"""

from curses import nonl
from glob import glob
import exudyn as exu
print("Using Exudyn version ", exu.__version__)
from exudyn.itemInterface import *
from exudyn.utilities import *

import numpy as np
print("Using Numpy version ", np.__version__)

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf


# global variable for external forces and moments (combined => efforts)
extEfforts = np.zeros(shape=(6,1))


# function to create and return a PoseStamped-ROS-msg
# coords: container with x, y and z coordinates
# :param euler: 
def createPoseStampedMsg(coords, euler):
    """
    function to create and return a PoseStamped-ros-msg

    :param coords: arbitrary container (tuple, list, np.array, ...) with x, y and z coordinates
    :param euler: arbitrary container (tuple, list, np.array, ...) with euler angles (pitch, roll and yaw)

    :return: the message object
    :rtype: geometry_msgs.msg.PoseStamped
    """

    # create message
    msg = PoseStamped()
    
    # write position into message
    msg.pose.position.x = coords[0]
    msg.pose.position.y = coords[1]
    msg.pose.position.z = coords[2]

    # endeffector should point straight down
    pitch = np.radians(euler[0])
    roll = np.radians(euler[1])
    yaw = np.radians(euler[2])

    # create Quaternion out of Euler angles
    quaternion = tf.transformations.quaternion_from_euler(pitch, yaw, roll)

    # only to be sure quaternion is correct
    assert np.linalg.norm(quaternion) == 1.0, "ERROR"

    # write orientation into message
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    # write current time into message
    msg.header.stamp = rospy.Time.now()

    return msg


def externalForceCallback(data):
    """
	callback function for external forces and moments, applied by the user on the robot

    every time new external efforts are registered by the robot(which happens continuously), 
    they are published and received by this callback function. Here they are written into 
    another variable, in order to be processed by exudyn

	:param geometry_msgs.msg.WrenchStamped data: received message
	"""

    print("Got efforts " + str(data.header.seq))

    # get forces
    fx = data.wrench.force.x
    fy = data.wrench.force.y
    fz = data.wrench.force.z

    # get moments
    mx = data.wrench.torque.x
    my = data.wrench.torque.y
    mz = data.wrench.torque.z

    # TODO: maybe apply low pass filter?

    # save for later use
    global extEfforts

    extEfforts[0] = fy  # TODO check for correct mapping
    extEfforts[1] = fz
    extEfforts[2] = fx
    extEfforts[3] = my
    extEfforts[4] = mz
    extEfforts[5] = mx


def main():

    # init ros
    rospy.init_node('ExudynExample1', anonymous=True)

    # publisher for pendulum poses (=endeffector positions)
    pub = rospy.Publisher('/my_cartesian_impedance_controller/setDesiredPose', PoseStamped, queue_size=1000)

    # subscriber for external forces
    rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, externalForceCallback)

    # init exudyn
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    tRes = 0.001 # step size in s
    tEnd = 1000     # simulation time in s

    g = 9.81    # gravity in m/s^2

    # start position of robot in robot base frame in meters
    globalStartPos = np.array([0.3, 0, 0.2])

    # matrix to transform simulation coordinates to robot world coordinates
    #trafoMat = np.array([[0, 0, 1],
    #                     [1, 0, 0],
    #                     [0, 1, 0]])

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # parameter and ground with rectangle visualisation
    # rigid pendulum:
    rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
    background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
    oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
    a = 0.5     #x-dim of pendulum
    b = 0.05    #y-dim of pendulum
    massRigid = 12
    inertiaRigid = massRigid/12*(2*a)**2

    # body consists out of Rigid2D Node (coordinates) and objekt RigidBody2D (physical properties and visualisation)
    graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[-1,0.5,-np.pi/2], initialVelocities=[0,0,5]));
    oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

    # create markers:
    # mG0 lays on the ground where the pendulum is connected
    mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-1,1.,0.]))

    # mR1 lays on the upper end of the pendulum where it is connected with ground
    mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5,0.,0.]))

    # mR2 lays in the middle of the pendulum where the gravitational force applies
    mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.]))

    # mR3 lays on the lower end of the pendulum where the user can interact
    mR3 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.5,0.,0.]))

    # a RevoluteJoint2D allows only rotation between markers mG0 and mR1
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))

    # gravitational force for pendulum
    mbs.AddLoad(Force(markerNumber=mR2, loadVector=[0, -massRigid*g, 0]))

    # external applied forces
    mbs.AddLoad(Force(markerNumber=mR3, loadVector=extEfforts[0:3]))    # TODO does this update when changing extEfforts?

    # external applied moments
    #mbs.AddLoad(Torque(markerNumber=mR3, loadVector=extEfforts[3:6]))   # TODO not sure if correct

    # sensor for position of endpoint of pendulum
    sensorPos = mbs.AddSensor(SensorMarker(markerNumber=mR3,
                                           outputVariableType=exu.OutputVariableType.Position))
    # store sensor value of each step in mbs variable, so that is accessible from user function
    mbs.variables['pos'] = sensorPos

    # initialisation of some variables
    posOffset = np.array([0, 0, 0]) # offset between user-interact position in simulation and robot world space 
    firstPose = True                # flag to determine first step; needed to to calculate posOffset
    T = np.eye(4)                   # for full coordinate transformation

    def PreStepUserFunction(mbs, t):
        nonlocal firstPose
        nonlocal posOffset
        nonlocal T

        # read current position
        pos_ = mbs.GetSensorValues(mbs.variables['pos'])
        pos = np.array(pos_)

        # in first iteration, calculate posOffset and T
        if firstPose:
            buf = [pos[2], pos[0], pos[1]]
            posOffset = globalStartPos - buf
            # full coordinate transformation
            posOffset = np.expand_dims(posOffset, axis=1)
            #T = np.concatenate((trafoMat, posOffset), axis=1)
            #T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
            firstPose = False

        # initilaize container
        posGlobal = [0, 0, 0]

        # transform simulation coordinates to robot world coordinates
        #pos = np.array([pos[0], pos[1], pos[2], 1])
        #posGlobal = T @ pos
        #posGlobal = posGlobal[:3]
        
        # as for now no rotation is required, it is faster to compute coordinates without matrix multiplication
        posGlobal[0] = pos[2] + posOffset[0][0]
        posGlobal[1] = pos[0] + posOffset[1][0]
        posGlobal[2] = pos[1] + posOffset[2][0]

        # compose message and publish
        msg = createPoseStampedMsg(posGlobal, (180, 0, 0))
        pub.publish(msg)

        # prestep-userfunction has to return true, else simulation stops
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
    simulationSettings.timeIntegration.simulateInRealtime = True    # crucial for operating with robot
    simulationSettings.displayStatistics = True
    simulationSettings.solutionSettings.solutionInformation = "2D Pendulum"

    # exudyn magic
    exu.StartRenderer()
    mbs.WaitForUserToContinue() # space/q to start - q to end
    exu.SolveDynamic(mbs, simulationSettings)
    #SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

    # remove "coordinatesSolution.txt" as it isn't needed
    import os
    os.remove("coordinatesSolution.txt")


if __name__ == "__main__":
    main()