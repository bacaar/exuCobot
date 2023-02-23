"""
Author: Aaron Bacher
Date: 14.02.2023

Combined Robot and VR Interface for Exudyn
"""

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray

import tf
from scipy.spatial.transform import Rotation

from exudyn.utilities import *

VR_POS_CORRECTION = np.array([0, -0.3, 0])
#VR_POS_CORRECTION = np.array([0, 0, 0])

class RobotVrInterface:

    # TODO: what if interfaceType==2 but robotInterface not available?
    def __init__(self, mbs, interfaceType, useImpedanceController=False) -> None:
        """
        Initialization

        :param int interfaceType: may be 1 if this instance should be the robot interface, 2 for the vr interface
        :param bool useImpedanceController: must be true if running robot controller is impedance controller. Default false for velocity controller
        """
        
        assert interfaceType == 1 or interfaceType == 2, "Undefined interface type"

        if interfaceType == 1:
            self.__robotInterface = RobotInterface(useImpedanceController)
            self.__interfaceType = 1
        else:
            self.__vrInterface = VrInterface(mbs, useImpedanceController)
            self.__interfaceType = 2

        if useImpedanceController:
            self.__topicBase = "/my_cartesian_impedance_controller"
        else:
            self.__topicBase = "/my_cartesian_velocity_controller"

        self.__rotationMatrix = np.eye(3)

    # actually only needed for vrInterface, but ODE coordinates must be consistent between vr and robot interface
    def setHand(self, mbs):

        if self.__interfaceType == 1:
            return mbs
        else:
            return self.__vrInterface.createEnvironment(mbs)

    def update(self, mbs, SC, t):
        """
        :return mbs
        """
        if self.__interfaceType == 1:
            return self.__robotInterface.update(mbs, t)
        else:
            return self.__vrInterface.update(mbs, SC, t)

    def getExternalEfforts(self):
        if self.__interfaceType == 1:
            return self.__robotInterface.getExternalEfforts()
        else:
            pass

    def getCurrentSystemState(self):
        if self.__interfaceType == 1:
            pass
        else:
            return self.__vrInterface.getCurrentSystemState()

    def setRotationMatrix(self, matrix):
        self.__rotationMatrix = matrix
        if self.__interfaceType == 1:
            pass
        else:
            return self.__vrInterface.setRotationMatrix(matrix)

    def setSettings(self, SC):
        if self.__interfaceType == 1:
            pass
        else:
            self.__vrInterface.setSettings(SC)

    def determineRobotStartPosition(self, interactionPointOffset=np.array([0,0,0])):
        """
        :param interactionPointOffset: offset from model origin to user interaction point in model
        """

        if self.__interfaceType == 1:
            return np.array([0,0,0])
        else:
            return self.__vrInterface.determineRobotStartPosition(interactionPointOffset)


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


class VrInterface:

    def __init__(self, mbs, useImpedanceController) -> None:

        self.__impedanceController = useImpedanceController
        self.__globalStartPos = np.array([0.0, 0.0, 0.0])
        #self.__globalStartPosSet = False

        self.__systemStateData = None

        self.__rotationMatrix = np.eye(3) # can be overwritten with setRotationMatrix()

        rospy.init_node('ExudynVrInterface', anonymous=True)
        # subscriber for current pose. Needed for localizing current end-effector position in vr-space
        if self.__impedanceController:
            self.__topicBase = "/my_cartesian_impedance_controller"
        else:
            self.__topicBase = "/my_cartesian_velocity_controller"

        self.__systemStateSub = rospy.Subscriber(self.__topicBase + "/SystemState", Float64MultiArray, self.__systemStateCallback)
        #globalStartPosSub = rospy.Subscriber(self.__topicBase + "/getCurrentPose", PoseStamped, self.__currentPoseCallback)

        # TODO: check if controller available
        # TODO: if yes, set origin relative to controller and robot configuration
        # TODO: check if tracker (for hand rendering) available
        pass

    def __del__(self):
        #self.__systemStateSub.unregister()
        pass

    def determineRobotStartPosition(self, interactionPointOffset=np.array([0,0,0])):
        """
        :param interactionPointOffset: offset from model origin to user interaction point in model
        """

        ## coordinates of controller in vr frame
        # for the moment this is hardcoded
        tc = np.array([1.38006377,  0.84095681,  0.2752991])
        tc = tc + VR_POS_CORRECTION             

        ## transformation matrix from controller to robot base in vr frame
        trb = np.array([0.86, -0.045, -0.43])

        ## listen to topic to get current robot end effector pose
        print("Locating robot in VR space")
        t0 = rospy.Time.now()

        eefPos = None

        def poseCallback(data):
            nonlocal eefPos
            
            if eefPos is None:
                eefPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

        eefPosSub = rospy.Subscriber(self.__topicBase + "/getCurrentPose", PoseStamped, poseCallback)

        # wait 5 seconds to locate global start position
        while eefPos is None:
            if rospy.Time.now() - t0 > rospy.Duration(5):
                print("ERROR: Did not get any robot position after 5 seconds of waiting")
                exit(-1)

        R = np.array([[-1, 0, 0],
                      [ 0, 0, 1],
                      [ 0, 1, 0]])
        te = R @ eefPos

        # subscriber isn't needed anymore
        eefPosSub.unregister()

        origin = self.__rotationMatrix @ (tc + trb + te - interactionPointOffset)

        self.__robotBase = self.__rotationMatrix @ (tc + trb)

        return origin

    def createEnvironment(self, mbs):

        ## get single values of VR_POS_COrrection
        dx, dy, dz = self.__rotationMatrix @ VR_POS_CORRECTION

        ## create floor
        plane = GraphicsDataQuad([[-4, 0, 0+dz],[ 1, 0, 0+dz],[ 1, 2, 0+dz],[-4, 2, 0+dz]],
                                 color4darkgrey, 
                                 nTiles=5,
                                 nTilesY=2,
                                 alternatingColor=color4lightgrey)
        mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                                   visualization=VObjectGround(graphicsData=[plane])))

        ## create table

        graphicsTable = GraphicsDataOrthoCube(xMin=-0.3, xMax=0.9,
                                              yMin=-0.4, yMax=0.4,
                                              zMin=-0.78, zMax=0,
                                              color=[0.7, 0.5, 0.3, 1])
        mbs.AddObject(ObjectGround(referencePosition=self.__robotBase,
                                   visualization=VObjectGround(graphicsData=[graphicsTable])))

        ## create Hand
        x = 0.08    # hand width
        y = 0.02    # hand length
        z = 0.15    # hand thickness

        graphicsHand = GraphicsDataOrthoCube(xMin=0, xMax=x,
                                             yMin=0, yMax=y,
                                             zMin=0, zMax=z,
                                             color=[0.7, 0.5, 0.3, 1])

        self.__oHand = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[graphicsHand])))

        return mbs

    def update(self, mbs, SC, time):

        ## get and apply simulation update from robot client
        data = self.getCurrentSystemState()

        if data is not None:
            mbs.systemData.SetSystemState(data)

        ## locate Tracker and move hand object there
        renderState = SC.GetRenderState()
        if renderState['openVR']['trackerPoses']:
            try:
                # get Tracker Pose
                T = renderState['openVR']['trackerPoses'][0].T

                # extract Orientation and Position from Tracker Pose
                R = self.__rotationMatrix @ HT2rotationMatrix(T)
                t = self.__rotationMatrix @ (HT2translation(T) + VR_POS_CORRECTION)

                mbs.SetObjectParameter(self.__oHand, "referencePosition", t)
                mbs.SetObjectParameter(self.__oHand, "referenceRotation", R)
            except Exception as e:
                #print(e)
                pass

        elif renderState['openVR']['controllerPoses']:
            try:
                # get Tracker Pose
                T = renderState['openVR']['controllerPoses'][0].T

                # extract Orientation and Position from Tracker Pose
                R = self.__rotationMatrix @ HT2rotationMatrix(T)
                t = self.__rotationMatrix @ (HT2translation(T) + VR_POS_CORRECTION)

                mbs.SetObjectParameter(self.__oHand, "referencePosition", t)
                mbs.SetObjectParameter(self.__oHand, "referenceRotation", R)
            except Exception as e:
                #print(e)
                pass

        else:
            pass

        return mbs


    def setRotationMatrix(self, matrix):
        self.__rotationMatrix = matrix


    # set visualisation settings for VR
    def setSettings(self, SC):
        SC.visualizationSettings.general.drawCoordinateSystem = True
        SC.visualizationSettings.window.startupTimeout = 100000 #wait ms for SteamVR
        SC.visualizationSettings.window.renderWindowSize= [1972, 2192]  # HMD render size
        SC.visualizationSettings.interactive.openVR.enable = True
        SC.visualizationSettings.interactive.openVR.showCompanionWindow = True
        SC.visualizationSettings.interactive.lockModelView = True #lock rotation/translation/zoom of model
        SC.visualizationSettings.interactive.openVR.logLevel = 3
        SC.visualizationSettings.general.graphicsUpdateInterval = 0.017
        SC.visualizationSettings.general.drawWorldBasis = True


    def getCurrentSystemState(self):
        return self.__systemStateData


    ## callbacks
    #def __currentPoseCallback(self, data):
    #    """
    #    callback function to set global robot position at beginning of program
    #    -> globalStartPos
    #    """
#
    #    if not self.__globalStartPosSet:
    #        self.__globalStartPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    #        self.__globalStartPosSet = True
    #        # TODO Logger
    #        print("Global start pos set")


    def __systemStateCallback(self, systemState):
        """
        callback function to get SystemData of RobotClient
        splits 1d array into multiple ones.

        Example structure for systemData if it would consist out of vector [x1, x2, x3] and [y1, y2]:
        [3, x1, x2, x3, 2, y1, y2]
        so every vector begins with its length
        """

        # amount of arrays in systemData. Default = 5
        nArrays = 5

        buf = [[] for i in range(nArrays)]
        
        currentList = 0
        index = 0
        
        for i in range(nArrays):
            currentListLength = int(systemState.data[index])
            for _ in range(currentListLength):
                index += 1
                buf[currentList].append(systemState.data[index])
            currentList += 1
            index +=1

        self.__systemStateData = buf


class RobotInterface:

    ## constructor
    def __init__(self, useImpedanceController):

        # this import is only needed on robot interface side; thus it is imported in this init 
        # function and not at the top of this file to not cause problem when using the VrInterface
        from util.msg import segmentCommand

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
        self.__robotCommandSendInterval = 0.02 #0.006    #s
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
            self.__commandClass = PoseStamped
        else:
            topicBase = '/my_cartesian_velocity_controller'
            brief = "VC"
            self.__commandClass = segmentCommand

        # publisher for pendulum poses (=endeffector positions)
        self.__pub = rospy.Publisher(topicBase + '/setTargetPose', self.__commandClass, queue_size=1000)

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
            self.__lastRobotCommandSentTime = t

        if t - self.__lastSystemStateUpdateTime >= self.__systemStateUpdateInterval:
            # publish system state vor vrInterface
            # TODO belongs in __publishSystemState()
            systemStateData = mbs.systemData.GetSystemState()
            systemStateList1d = []
            for array in systemStateData:
                systemStateList1d.append(float(len(array)))
                for i in range(len(array)):
                    systemStateList1d.append(array[i])

            self.__publishSystemState(systemStateList1d)
            self.__lastSystemStateUpdateTime = t

        return mbs


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
            msg = self.__commandClass()
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