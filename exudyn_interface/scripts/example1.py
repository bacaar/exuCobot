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

import rospy
from geometry_msgs.msg import PoseStamped
import tf

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


def main():

    # init ros
    rospy.init_node('ExudynExample1', anonymous=True)
    pub = rospy.Publisher('/my_cartesian_impedance_controller/setDesiredPose', PoseStamped, queue_size=1000)

    # erstelle Systemcontainer und
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    tRes = 0.001 # step size
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
            buf = [pos[2], pos[0], pos[1]]
            posOffset = globalStartPos - buf
            # full coordinate transformation
            posOffset = np.expand_dims(posOffset, axis=1)
            #T = np.concatenate((rotMat, posOffset), axis=1)
            #T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)


        posGlobal = [0, 0, 0]
        
        posGlobal[0] = pos[2] + posOffset[0][0]
        posGlobal[1] = pos[0] + posOffset[1][0]
        posGlobal[2] = pos[1] + posOffset[2][0]

        #print(i, posGlobal, t)
        i += 1

        msg = createPoseStampedMsg(posGlobal, (180, 0, 0))
        pub.publish(msg)

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