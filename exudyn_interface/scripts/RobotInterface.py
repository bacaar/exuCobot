#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 20.12.2022

Ros (Robot) Interface for Exudyn
"""

import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
from util.msg import segmentCommand

import tf
from scipy.spatial.transform import Rotation


def createPoseStampedMsg(coords, euler, time):
    """
    function to create and return a PoseStamped-ros-msg

    :param coords: arbitrary container (tuple, list, np.array, ...) with x, y and z coordinates
    :param euler: arbitrary container (tuple, list, np.array, ...) with euler angles (pitch, roll and yaw)
    :param time: instance of ros class "Time" with time for message

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
    #norm = np.linalg.norm(quaternion)
    #assert abs(1-norm) <= 0.01, "ERROR calculating Quaternion, norm is " + str(norm)

    # write orientation into message
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    # write current time into message
    msg.header.stamp = time

    return msg


class RobotInterface:

    ## constructor
    def __init__(self, useImpedanceController):

        # initialize some variables

        # force calibration
        self.__effortsCalibrated = False
        self.__nDesiredCalibrationValues = 100    # external efforts are sent with 30Hz from the robot, so calibration progress takes nCalibrationValues/30 seconds to finish
        self.__calibrationValues = np.zeros(shape=(self.__nDesiredCalibrationValues, 6))
        self.__nGotCalibrationValues = 0   # iteration variable

        # global variable for external forces and moments (combined => efforts)
        self.__extEfforts = np.zeros(shape=(6, 1))

        # measured force and torque by robot are not equal zero at rest, so store first measured force and torque (assumption: at rest) and substract them of every other one measured
        self.__effortsOffset = np.zeros(shape=(6, 1))
        self.__effortsThreshold = np.zeros(shape=(6, 1))
        self.__effortsThresholdFactor = 10  # factor for scaling threshold

        # start position of robot in robot base frame in meters
        self.__globalStartPos = np.array([0.4, 0, 0.2])    # default, but not precise enough
        self.__globalStartPosSet = False

        self.__posOffset = np.array([0, 0, 0]) # offset between user-interact position in simulation and robot world space 
        self.__firstPose = True                # flag to determine first step; needed to calculate posOffset
        #self.__T = np.eye(4)                   # for full coordinate transformation; currently not used

        self.__impedanceController = useImpedanceController

        # timing variable to know when to send new command to robot or when to publish new mbs system state update
        self.__robotCommandSendInterval = 0.006    #s
        self.__lastRobotCommandSentTime = -self.__robotCommandSendInterval
        self.__systemStateUpdateInterval = 0.017  #s
        self.__lastSystemStateUpdateTime = -self.__systemStateUpdateInterval

        self.__lastStepTime = 0    # last step time (exudyn time)

        self.__logFile = None

        # init ros
        rospy.init_node('ExudynRobotInterface', anonymous=True)

        # depending on used controller, different topics have to be used
        if self.__impedanceController:
            topicBase = '/my_cartesian_impedance_controller'
            brief = "IC"
            classToUse = PoseStamped
        else:
            topicBase = '/my_cartesian_velocity_controller'
            brief = "VC"
            classToUse = segmentCommand

        # publisher for pendulum poses (=endeffector positions)
        self.__pub = rospy.Publisher(topicBase + '/setTargetPose', classToUse, queue_size=1000)

        # publisher for system data
        self.__pubS = rospy.Publisher(topicBase + '/SystemState', Float64MultiArray, queue_size=1)
        
        # publisher for filtered force
        self.__pubF = rospy.Publisher(topicBase + '/analysis/getRegisteredForce', WrenchStamped, queue_size=10)

        # subscriber for current pose
        globalStartPosSub = rospy.Subscriber(topicBase + "/getCurrentPose", PoseStamped, self.__currentPoseCallback)

        # subscriber for external forces
        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.__externalEffortCallback)

        # open and init log file (csv)
        self.__logFile = open("/home/robocup/catkinAaron/src/exuCobot/log/exudyn" + brief + ".csv", "w")
        
        self.__logFile.write("rt,dt,px,py,pz,vx,vy,vz,ax,ay,az\n")

        # setup transform listener for transformation from endeffector to world frame
        self.__tfListener = tf.TransformListener()

        # calibrate robot
        print("Calibrating. Do not touch robot")
        
        # wait for global start position
        print("Waiting for global start position")
        t0 = rospy.Time.now()

        while not self.__globalStartPosSet:
            if rospy.Time.now() - t0 > rospy.Duration(5):
                print("ERROR: Did not get any robot position after 5 seconds of waiting")
                exit(-1)

        globalStartPosSub.unregister()  # we don't need current pose anymore

        # wait for calibration to finish
        print("Waiting for effort calibration")
        while not self.__effortsCalibrated:
            pass

        print("Efforts calibrated")
        print("Everything ready to go, start using robot now")


    ## destructor
    def __del__(self):
        self.__logFile.close()

    # getter
    def getExternalEfforts(self):
        return self.__extEfforts

    
    def update(self, mbs, t):

        # publishing each and every step is too much, this slows the connection down
        # thus publish every xth pose, only
        # furthermore, as vrInterface is only updating the graphics with f=60Hz, we don't have to update
        # system state every 1ms, so with f=1000Hz. Instead f=60Hz equivalents to update every 1/60=17ms

        if t - self.__lastRobotCommandSentTime >= self.__robotCommandSendInterval:
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

            # calculate angle
            angleX = float(round(180+np.rad2deg(rot[0]), 4))

            #print(angleX, type(angleX))

            self.__publishRobotCommand(pos, vel, acc, angleX, t)
            
            lastRobotCommandSentTime = t

        if t - self.__lastSystemStateUpdateTime >= self.__systemStateUpdateInterval:
            # publish system state vor vrInterface
            systemStateData = mbs.systemData.GetSystemState()
            systemStateList1d = []
            for array in systemStateData:
                systemStateList1d.append(float(len(array)))
                for i in range(len(array)):
                    systemStateList1d.append(array[i])

            self.__publishSystemState(systemStateList1d)
            lastSystemStateUpdateTime = t


    def __publishSystemState(self, systemData): 
        #dataList = []   
        msg = Float64MultiArray()
        
        #msg.layout.dim = 5 
        #for item in systemData: 
        #    dataList += [item]


        """for i in len(systemData): 
            dataList += []
            for j in len(systemData[i]): 
                dataList[i] +=  [float(systemData[i][j])]"""
        # print(dataList)
        # dataList = [[1,2,3], [4,5,7], [8,9,10]]
        # dataList=[[float(dataList[j][i]) for i in range(len(dataList[j]))] for j in range(len(dataList))]
        #print(dataList)
        msg.data = systemData # dataList
        self.__pubS.publish(msg)


    def __publishRobotCommand(self, posExu, vel, acc, angleX, tExu):

        # in first iteration, offset between exudyn local position and robot global position has to be determined
        if self.__firstPose:

            self.__posOffset = self.__globalStartPos - posExu
            self.__posOffset = np.expand_dims(self.__posOffset, axis=1)

            # full coordinate transformation
            # TODO: don't forget rotation
            #T = np.concatenate((trafoMat, posOffset), axis=1)
            #T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
            self.__firstPose = False

        # local exudyn position has to be transformed to global robot position
        # initialize container
        pos = [0, 0, 0]

        # as for now no rotation is required, it is faster to compute coordinates without matrix multiplication
        pos[0] = posExu[0] + self.__posOffset[0][0]
        pos[1] = posExu[1] + self.__posOffset[1][0]
        pos[2] = posExu[2] + self.__posOffset[2][0]

        # compose message and publish
        tRos = rospy.Time.now()
        segmentDuration = tExu - self.__lastStepTime     # time from last position to this position
        #print(segment_duration.to_sec(), "\t", t - self.lastStepTime)
        self.__lastStepTime = tExu

        if self.__impedanceController:
            msg = createPoseStampedMsg(pos, (angleX, 0, 0), tRos)
        else:
            msg = segmentCommand()
            msg.x.pos = pos[0]
            msg.y.pos = pos[1]
            msg.z.pos = pos[2]

            msg.x.vel = vel[0]
            msg.y.vel = vel[1]
            msg.z.vel = vel[2]

            msg.x.acc = acc[0]
            msg.y.acc = acc[1]
            msg.z.acc = acc[2]

            msg.dt = segmentDuration

        self.__pub.publish(msg)

        self.__logFile.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(tRos.to_sec(), segmentDuration, posExu[0], posExu[1], posExu[2], vel[0], vel[1], vel[2], acc[0], acc[1], acc[2]))


    ## callbacks
    def __currentPoseCallback(self, data):
        """
        callback function to set global robot position at beginning of program
        -> globalStartPos
        """

        if not self.__globalStartPosSet:
            self.__globalStartPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
            self.__globalStartPosSet = True
            # TODO Logger
            print("Global start pos set")


    def __externalEffortCallback(self, data):
        """
        callback function for external forces and torques, applied by the user on the robot
        for simplicity reasons, forces and torques are denoted as efforts here

        every time new external efforts are registered by the robot(which happens continuously), 
        they are published and received by this callback function. Here they are written into 
        a global variable, in order to be processed by exudyn

        :param geometry_msgs.msg.WrenchStamped data: received message
        """

        # for some reason, at the first few (3-4) calls of this callback-method, the frames panda_link0 is not known to tf
        # thus whole function is under try-statement, just skip those first calls
        try:
            # transform forces and torques from K frame (endeffector, flange) (=panda_hand) into world frame (panda_link0)

            #                                            child          parent      use latest update
            (_,rot) = self.__tfListener.lookupTransform('/panda_link0', '/panda_K', rospy.Time(0))

            R = np.array(Rotation.from_quat(rot).as_matrix())
            # print("Transformation matrix (rot 3x3) determinant: ", R)

            # K frame
            forceK = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
            torqueK = np.array([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

            # world frame
            forceW = R @ forceK
            torqueW = R @ torqueK

            # as robot measures efforts it needs to compensate external force, negative efforts are the ones applied (TODO lookup if correct)
            forceW *= -1
            torqueW *= -1

            # before operation, calibrate forces
            if not self.__effortsCalibrated:

                # collecting values
                if self.__nGotCalibrationValues < self.__nDesiredCalibrationValues:

                    self.__calibrationValues[self.__nGotCalibrationValues] = np.array([forceW[0],
                                                                                       forceW[1],
                                                                                       forceW[2],
                                                                                       torqueW[0],
                                                                                       torqueW[1],
                                                                                       torqueW[2]])
                    self.__nGotCalibrationValues += 1

                # evaluating values
                else:
                    for i in range(6):
                        
                        # use mean value as offset
                        self.__effortsOffset[i] = np.mean(self.__calibrationValues[:,i])

                        # use peak to peak value as threshold
                        self.__effortsThreshold[i] = (np.max(self.__calibrationValues[:,i]) - np.min(self.__calibrationValues[:,i]))*self.__effortsThresholdFactor

                    self.__effortsCalibrated = True

            # operating mode
            else:
                
                efforts = np.zeros(shape=(6,))

                # get forces and substract offset
                efforts[0] = forceW[0] - self.__effortsOffset[0]
                efforts[1] = forceW[1] - self.__effortsOffset[1]
                efforts[2] = forceW[2] - self.__effortsOffset[2]

                # get torques and substract offset
                efforts[3] = torqueW[0] - self.__effortsOffset[3]
                efforts[4] = torqueW[1] - self.__effortsOffset[4]
                efforts[5] = torqueW[2] - self.__effortsOffset[5]

                # additional to force offset, there is also some noise on force/torque measurement which have to be eliminated with threshold

                effortNames = ["fx", "fy", "fz", "mx", "my", "mz"]

                for i in range(6):
                    if abs(efforts[i]) > self.__effortsThreshold[i]:
                        self.__extEfforts[i] = efforts[i]
                        #print("effort", effortNames[i], "=", extEfforts[i])
                    else:
                        self.__extEfforts[i] = 0
                # TODO effort trafo with rot matrix

                # publish registered external force
                msgF = WrenchStamped()
                msgF.header.stamp = rospy.Time.now()
                msgF.wrench.force.x = self.__extEfforts[0]
                msgF.wrench.force.y = self.__extEfforts[1]
                msgF.wrench.force.z = self.__extEfforts[2]
                self.__pubF.publish(msgF)
        
        except Exception as e:
            #print(e)
            pass