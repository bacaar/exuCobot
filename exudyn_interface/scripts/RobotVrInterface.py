"""
Author: Aaron Bacher
Date of creation: 14.02.2023
Last modified: 01.03.2023

Combined Robot and VR Interface for Exudyn
"""

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray

import tf
from scipy.spatial.transform import Rotation

from exudyn.utilities import *

## define some hardcoded variables. All needed for vrInterface only

# position of handheld controller, which is mounted on robot table and thus indicates (relative) robot position
HHC_POS_IN_VR_FRAME = np.array([1.38436, 0.833307, 0.326828])

# translation from above named hand held controller to robot base frame
HHC_TO_ROBOT_BASE = np.array([0.86, -0.045, -0.43])	# =~ position of robot on table

# apparently above created robot position is not very intuitive
# with this variable position can be further translated
VR_POS_CORRECTION = np.array([0, -0.25, 0])

HAND_MODEL_OFFSET = [0.02, 0.06, 0.00]

# desired frames per second
VR_FPS = 60

##

def parseArgv(argv):
    """
    Function to parse program arguments

    Args:
        argv (list of strings): program arguments from sys.argv

    Returns:
        tuple (int, bool): parameters needed to initialize RobotVrInterface 
    """

    # default values
    client = None   # is mandatory to be specified in program arguments
    useImpedanceController = False  # default: velocity controller

    # just to be sure, that filename (argv[0]) can't trigger any option below
    filename = argv[0]
    argv = argv[1:]

    def printUsage():
        print("Usage: python3 " + filename + " [clientType vr/robot] [controllerType -v/-i]")

    if "r" in argv or "robot" in argv:
        client = 1
    if "v" in argv or "vr" in argv:
        if client == 1: # in case "r"/"robot" and "v"/"vr" are both in argv
            printUsage()
            exit(-1)
        else:
            client = 2
    if "-i" in argv:
        useImpedanceController = True
    if "-v" in argv:
        if useImpedanceController == True:  # in case -i and -v are both in argv
            printUsage()
            exit(-1)
        else:
            pass # velocity controller already in use by default
    if "-h" in argv or "--help" in argv:
        printUsage()
        exit(0)

    if client == None:
        printUsage()
        exit(-1)

    return client, useImpedanceController
    

class RobotVrInterface:
    """
    client for accessing VR or robot from Python
    holds interface to either robot or VR functionalities
    thus does not anything by itself, just calls respective methods of robot or vr interfaces
    """

    def __init__(self, clientType, useImpedanceController=False) -> None:
        """
        Initialization method for RobotVrInterface

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
            clientType (int): may be 1 if this instance should be the robot interface, 2 for the vr interface
            useImpedanceController (bool, optional): flag to use impedance controller. Defaults to False (velocity controller).
        """
        
        # make sure client type is valid
        assert clientType == 1 or clientType == 2, "Undefined interface type"

        # depending on client type, setup attributes
        if clientType == 1:
            self.__robotInterface = RobotInterface(useImpedanceController)
            self.__clientType = 1
        else:
            self.__vrInterface = VrInterface(useImpedanceController)
            self.__clientType = 2

        # depending on used controller, set baseName for ros-topics
        if useImpedanceController:
            self.__topicBase = "/exucobot_cartesian_impedance_controller"
        else:
            self.__topicBase = "/exucobot_cartesian_velocity_controller"

        # just for the case that the rotation matrix isn't specified by the user aftewards, set it to identity-matrix as default
        self.setRotationMatrix(np.eye(3))

        self.__tRes = 0.001    # step size in s
        self.__tEnd = 10000

    
    def createEnvironment(self, mbs):
        """
        method to create environment (tables, floor, hand ecc.) in vr client

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
        """

        if self.__clientType == 1:
            pass
        else:
            self.__vrInterface.createEnvironment(mbs)

    
    def setUIP(self, marker, mbs):
        """
        method to set UIP in model and to add cllback function to apply effforts on it

        Args:
            marker (exudyn.exudynCPP.MarkerIndex): Exudyn marker at UIP position
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
        """

        if self.__clientType == 1:
            self.__robotInterface.setUIP(marker, mbs)
        else:
            pass



    def update(self, mbs, SC, t):
        """
        method to be called once per frame/control cyle in Exudyn PreStepUserFunction
        needed by both interfaces

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
            SC (exudyn.exudynCPP.SystemContainer): Exudyn system container
            t (float): elapsed time since simulation start
        """

        if self.__clientType == 1:
            self.__robotInterface.update(mbs, t)
        else:
            self.__vrInterface.update(mbs, SC)
        

    def simulate(self, mbs, SC, simulationSettings):
        """_summary_

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
            SC (exudyn.exudynCPP.SystemContainer): Exudyn system container
            simulationSettings (exudyn.exudynCPP.SimulationSettings): Exudyn simulation settings
        """

        self.__setSettings(SC)   

        # simulate for a very long time in real time
        simulationSettings.timeIntegration.endTime = self.__tEnd
        simulationSettings.timeIntegration.numberOfSteps = int(self.__tEnd/self.__tRes)
        simulationSettings.timeIntegration.simulateInRealtime = True    # crucial for operating with robot

        # start rendering
        exu.StartRenderer()

        if self.__clientType == 1:
            self.__robotInterface.simulate(mbs, simulationSettings)
        else:
            self.__vrInterface.simulate(mbs, SC)

        exu.StopRenderer() #safely close rendering window!


    def __setSettings(self, SC):
        """
        method to set specified visualization settings for vr-view

        Args:
            SC (exudyn.exudynCPP.SystemContainer): Exudyn system container
        """

        # set common settings for both robot and vr interface
        SC.visualizationSettings.general.autoFitScene = False

        # individual settings
        if self.__clientType == 1:
            pass # nothing to do here
        else:
            self.__vrInterface.setSettings(SC)


    def getExternalEfforts(self):
        """
        getter-method for external efforts
        to be called from loadUserFunctions in Exudyn Python-code
        only needed by robot interface

        Returns:
            np.array[float64], shape=(6,): efforts [fx, fy, fz, mx, my, mz]
        """

        if self.__clientType == 1:
            return self.__robotInterface.getExternalEfforts()
        else:
            return [0, 0, 0, 0, 0, 0]


    def setRotationMatrix(self, matrix):
        """
        setter-method for rotation (view) matrix
        if matrix != np.eye(3), must be called before determineRobotStartPosition(...)
        only relevant for vr interface

        Args:
            matrix (np.array[float64] shape=(3,3)): rotation matrix
        """

        self.__rotationMatrix = matrix
        if self.__clientType == 1:
            pass
        else:
            self.__vrInterface.setRotationMatrix(matrix)


    def determineRobotStartPosition(self, interactionPointOffset=np.array([0,0,0])):
        """
        locates robot in space and returns sim-model origin in VR-space
        controller-pos are hardcoded and must be adapted if robot/table is moved!

        Args:
            interactionPointOffset (np.array[float64], shape=(3,), optional): offset from model (not VR!) origin to user interaction point in model. Defaults to np.array([0,0,0]).

        Returns:
            np.array[float64], shape=(3,): origin of sim-model in VR-space
        """

        if self.__clientType == 1:
            # not relevant for robot interface
            return np.array([0,0,0])
        else:
            return self.__vrInterface.determineRobotStartPosition(interactionPointOffset)


def createPoseStampedMsg(coords, euler, time):
    """
    function to create and return a PoseStamped-ros-msg

    Args:
        coords (arbitrary container, shape=(3,)): x, y and z coordinates
        euler (arbitrary container, shape=(3,)): euler angles (pitch, roll and yaw)
        time (rospy.Time): time for message

    Returns:
        geometry_msgs.msg.PoseStamped: the message object
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

    # write orientation into message
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    # write current time into message
    msg.header.stamp = time

    return msg


def createTable(mbs, pos, dim, tableTopThickness, tableBaseThickness, tableTopColor, tableBaseColor):
    """_summary_

    Args:
        mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
        pos (arbitrary container, shape=(3,)): one of the (bottom) corners of the table, seen as a unit (base and top together)
        dim (arbitrary container, shape=(3,)): dimensions of the table, seen as a unit (base and top together)
        tableTopThickness (float): thickness of table top (plate)
        tableBaseThickness (float): thickness of table base (legs)
        tableTopColor (arbitrary container, shape=(4,)): color of table top (plate)
        tableBaseColor (arbitrary container, shape=(4,)): color of table base (legs)

    Returns:
        exudyn.exudynCPP.MainSystem: modified mbs
    """

    # determin base/leg lengths
    tableBaseLength = dim[2]-tableTopThickness

    # create table top
    graphicsTableTop = GraphicsDataOrthoCube(xMin=0, xMax=dim[0],
                                             yMin=0, yMax=dim[1],
                                             zMin=tableBaseLength, zMax=dim[2],
                                             color=tableTopColor)
    mbs.AddObject(ObjectGround(referencePosition=pos,
                               visualization=VObjectGround(graphicsData=[graphicsTableTop])))

    # create table base
    graphicsTableBase = GraphicsDataOrthoCube(xMin=0, xMax=tableBaseThickness,
                                              yMin=0, yMax=tableBaseThickness,
                                              zMin=0, zMax=tableBaseLength,
                                              color=tableBaseColor)
    mbs.AddObject(ObjectGround(referencePosition=pos,
                               visualization=VObjectGround(graphicsData=[graphicsTableBase])))
    mbs.AddObject(ObjectGround(referencePosition=pos+np.array([dim[0]-tableBaseThickness, 0, 0]),
                               visualization=VObjectGround(graphicsData=[graphicsTableBase])))
    mbs.AddObject(ObjectGround(referencePosition=pos+np.array([0, dim[1]-tableBaseThickness, 0]),
                               visualization=VObjectGround(graphicsData=[graphicsTableBase])))
    mbs.AddObject(ObjectGround(referencePosition=pos+np.array([dim[0]-tableBaseThickness, dim[1]-tableBaseThickness, 0]),
                               visualization=VObjectGround(graphicsData=[graphicsTableBase])))


class VrInterface:

    def __init__(self, useImpedanceController) -> None:
        """
        constructor for VrInterface

        Args:
            useImpedanceController (bool): flag to wheter use impedance controller (true) or velocity controller (false)
        """

        # set some class attributes
        self.__impedanceController = useImpedanceController
        self.__globalStartPos = np.array([0.0, 0.0, 0.0])

        self.__systemStateData = None
        self.__currentSimulationTime = 0

        # init ROS node
        rospy.init_node('ExudynVrInterface', anonymous=True)
        
        # subscriber for current pose. Needed for localizing current end-effector position in vr-space
        if self.__impedanceController:
            self.__topicBase = "/exucobot_cartesian_impedance_controller"
        else:
            self.__topicBase = "/exucobot_cartesian_velocity_controller"

        self.__systemStateSub = rospy.Subscriber(self.__topicBase + "/systemState", Float64MultiArray, self.__systemStateCallback)


    def determineRobotStartPosition(self, interactionPointOffset=np.array([0,0,0])):
        """
        locates robot in space and returns sim-model origin in VR-space
        controller-pos are hardcoded and must be adapted if robot/table is moved!

        Args:
            interactionPointOffset (np.array[float64], shape=(3,), optional): offset from model (not VR!) origin to user interaction point in model. Defaults to np.array([0,0,0]).

        Returns:
            np.array[float64], shape=(3,): origin of sim-model in VR-space
        """

        ## coordinates of controller in vr frame
        tc = HHC_POS_IN_VR_FRAME
        tc = tc + VR_POS_CORRECTION             

        ## transformation matrix from controller to robot base in vr frame
        trb = HHC_TO_ROBOT_BASE

        ## listen to topic to get current robot end effector pose
        print("Locating robot in VR space")
        t0 = rospy.Time.now()

        eefPos = None

        def poseCallback(data):
            nonlocal eefPos
            
            if eefPos is None:
                eefPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

        eefPosSub = rospy.Subscriber(self.__topicBase + "/currentPose", PoseStamped, poseCallback)

        # wait 5 seconds to locate global start position
        while eefPos is None:
            if rospy.Time.now() - t0 > rospy.Duration(5):
                print("ERROR: Did not get any robot position after 5 seconds of waiting")
                exit(-1)

        R = np.array([[-1, 0, 0],
                      [ 0, 0, 1],
                      [ 0, 1, 0]])
        te = R @ eefPos

        # combine everything
        origin = self.__rotationMatrix @ (tc + trb + te - interactionPointOffset)

        # store robotBase-coordinates for further use
        self.__robotBase = self.__rotationMatrix @ (tc + trb)

        return origin
     

    def createEnvironment(self, mbs):
        """
        method to create environment (tables, floor, hand ecc.)

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn

        Returns:
            exudyn.exudynCPP.MainSystem: modified mbs must be returned
        """

        ## get VR_POS_CORRECTION in vr coordinates
        corr = self.__rotationMatrix @ VR_POS_CORRECTION

        ## create lamp
        lamp = GraphicsDataSphere(color=color4yellow)
        mbs.AddObject(ObjectGround(referencePosition=[-1,1.5,3],
                                   visualization=VObjectGround(graphicsData=[lamp])))

        ## create world borders

        # decide wheter to create platform (ground with abyss) or room (floor with walls)
        createRoom = True

        # create room
        if createRoom:
            height = np.array([0., 0., 3.])
            col1 = color4white
            col2 = color4white
            x, y, z = self.__robotBase
            # x-... and y-... are distances from robot to room wall
            cornerPoints = np.array([[x-1, y-0.94, 0.],[ 1., y-0.94, 0.],[ 1., 2., 0.],[x-1, 2., 0.]])

        # create abyss
        else:
            height = np.array([0., 0., -20.])
            col1 = color4darkgrey
            col2 = color4lightgrey
            cornerPoints = np.array([[-2., 0., 0.],[ 1., 0., 0.],[ 1., 2., 0.],[-2., 2., 0.]])

        for i in range(4):
            cornerPoints[i] += corr

        # create floor
        nTilesX = int(abs(cornerPoints[0][0]-cornerPoints[1][0]))
        nTilesY = int(abs(cornerPoints[0][1]-cornerPoints[2][1]))

        plane = GraphicsDataQuad(cornerPoints,
                                 color4lightgrey, 
                                 nTiles=nTilesX,
                                 nTilesY=nTilesY,
                                 alternatingColor=color4lightgrey)
        mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                                   visualization=VObjectGround(graphicsData=[plane])))
        
        # create walls / abyss
        for i in range(4):
            j = i+1
            if j == 4:
                j = 0

            if i%2==0:
                nTilesX = int(abs(cornerPoints[i][0]-cornerPoints[j][0]))
            else:
                nTilesX = int(abs(cornerPoints[i][1]-cornerPoints[j][1]))
            nTilesY = int(abs(height[2]))

            abyssCorners = np.array([cornerPoints[i], cornerPoints[j], cornerPoints[j]+height, cornerPoints[i]+height])
            plane = GraphicsDataQuad(abyssCorners,
                                    col1, 
                                    nTiles=nTilesX,
                                    nTilesY=nTilesY,
                                    alternatingColor=col2)
            mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                                    visualization=VObjectGround(graphicsData=[plane])))


        ## create table
        createTable(mbs, self.__robotBase+np.array([-0.3, -0.4, -0.78]), np.array([1.2, 0.8, 0.78]), 0.1, 0.08, [0.3, 0.3, 0.3, 1], [0.75, 0.75, 0.75, 1])
        createTable(mbs, self.__robotBase+np.array([0.9, -0.94, -0.78]), np.array([2, 1, 0.72]), 0.025, 0.045, [1, 0.8, 0.4, 1], [0.75, 0.75, 0.75, 1])

        ## create Hand
        graphicsHand = GraphicsDataFromSTLfile("hand_stl/Hand_R_centered_2.stl",    # TODO: this is not very flexible (does not work when file called from another directory)
                                               color=[0.95, 0.8, 0.8, 1],   # skin-color / pink
                                               scale=0.001,
                                               Aoff=np.eye(3) @ RotationMatrixZ(np.pi/8) @ RotationMatrixY(np.pi),
                                               pOff=HAND_MODEL_OFFSET)

        self.__oHand = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[graphicsHand])))
    

    def update(self, mbs, SC):
        """
        method to be called once per frame/control cyle in Exudyn PreStepUserFunction
        moves hand object to correct position
        loads current system State (received from robot client)

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
            SC (exudyn.exudynCPP.SystemContainer): Exudyn system container

        Returns:
            exudyn.exudynCPP.MainSystem: modified mbs must be returned
        """

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
    

    def simulate(self, mbs, SC):
        """
        While the robotInterface really simulates the mbs, the VR interface only updates the vizualization

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
            SC (exudyn.exudynCPP.SystemContainer): Exudyn system container
        """

        simRunning = True
        timout = 1/VR_FPS

        while simRunning:

            # update visualization data
            mbs.systemData.SetSystemState(self.getCurrentSystemState(), exu.ConfigurationType.Visualization)
            mbs.systemData.SetTime(self.__currentSimulationTime, exu.ConfigurationType.Visualization)

            self.update(mbs, SC)

            # update screen
            mbs.SendRedrawSignal()

            # do whatever is needed in between steps
            exu.DoRendererIdleTasks(timout)

            # check wheter to continue or stopping visualizing
            simRunning = not mbs.GetRenderEngineStopFlag()  # user can stop program by pressing 'q'


    def setRotationMatrix(self, matrix):
        """
        setter-method for model-rotation / view matrix

        Args:
            matrix (np.ndarray[float64], shape=(3,3)): rotation matrix
        """

        self.__rotationMatrix = matrix


    def setSettings(self, SC):
        """
        method to set specified visualization settings for vr-view

        Args:
            SC (exudyn.exudynCPP.SystemContainer): Exudyn system container
        """

        SC.visualizationSettings.general.drawCoordinateSystem = False
        SC.visualizationSettings.general.graphicsUpdateInterval = 1/VR_FPS # = 60 Hz

        SC.visualizationSettings.window.startupTimeout = 100000 # wait ms for SteamVR
        SC.visualizationSettings.window.limitWindowToScreenSize = False # needed for screen size being smaller than HMD display size
        SC.visualizationSettings.window.renderWindowSize = [1972, 2192]  # HMD render size

        SC.visualizationSettings.interactive.lockModelView = True # lock rotation/translation/zoom of model
        SC.visualizationSettings.interactive.openVR.enable = True
        SC.visualizationSettings.interactive.openVR.showCompanionWindow = True
        SC.visualizationSettings.interactive.openVR.logLevel = 3
        
        SC.visualizationSettings.openGL.enableLighting = True
        SC.visualizationSettings.openGL.enableLight0 = True
        SC.visualizationSettings.openGL.light0position = [-1,1.5,3,0]
        SC.visualizationSettings.openGL.shadow = 0.5


    def getCurrentSystemState(self):
        """
        getter-method: returns current system data (updated by __systemStateCallback(...))

        Returns:
            list: Exudyn system state
        """
        return self.__systemStateData


    def __systemStateCallback(self, systemState):
        """
        callback method to get SystemData of robotClient
        splits 1d array into multiple ones.

        Args:
            systemState (std_msgs.msg.Float64MultiArray): Exudyn systemData state
        """

        # Example structure for systemData if it would consist out of vector [x1, x2, x3] and [y1, y2]:
        # [3, x1, x2, x3, 2, y1, y2]
        # so every vector begins with its length

        # however, before systemData itself: first data of vector is simTime
        self.__currentSimulationTime = systemState.data[0]

        # amount of arrays in systemData: 5
        nArrays = 5

        buf = [[] for i in range(nArrays)]
        
        currentList = 0
        index = 1
        
        # iterate over message data and write it into buf
        for i in range(nArrays):
            currentListLength = int(systemState.data[index])
            for _ in range(currentListLength):
                index += 1
                buf[currentList].append(systemState.data[index])
            currentList += 1
            index +=1

        # store acquired data for later
        self.__systemStateData = buf


class RobotInterface:

    def __init__(self, useImpedanceController):
        """
        constructor for RobotInterface

        Args:
            useImpedanceController (bool): flag to wheter use impedance controller (true) or velocity controller (false)
        """

        # this import is only needed on robot interface side; thus it is imported in this init 
        # method and not at the top of this file to not cause problems when using the VrInterface
        from util.msg import segmentCommand

        # initialize some attributes

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

        self.__impedanceController = useImpedanceController

        # timing variable to know when to send new command to robot or when to publish new mbs system state update
        self.__robotCommandSendInterval = 0.02 #0.006    #s
        self.__lastRobotCommandSentTime = -self.__robotCommandSendInterval
        self.__systemStateUpdateInterval = 1/VR_FPS  #s
        self.__lastSystemStateUpdateTime = -self.__systemStateUpdateInterval

        self.__lastStepTime = 0    # last step time (exudyn time)

        self.__logFile = None

        # init ros node
        rospy.init_node('ExudynRobotInterface', anonymous=True)

        # depending on used controller, different topics have to be used
        if self.__impedanceController:
            topicBase = '/exucobot_cartesian_impedance_controller'
            brief = "IC"
            self.__commandClass = PoseStamped
        else:
            topicBase = '/exucobot_cartesian_velocity_controller'
            brief = "VC"
            self.__commandClass = segmentCommand

        # publisher for pendulum poses (=endeffector positions)
        self.__pub = rospy.Publisher(topicBase + '/referencePose', self.__commandClass, queue_size=1000)

        # publisher for system data
        self.__pubS = rospy.Publisher(topicBase + '/systemState', Float64MultiArray, queue_size=1)
        
        # publisher for filtered force
        self.__pubF = rospy.Publisher(topicBase + '/analysis/registeredForce', WrenchStamped, queue_size=10)

        # subscriber for current pose
        globalStartPosSub = rospy.Subscriber(topicBase + "/currentPose", PoseStamped, self.__currentPoseCallback)

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


    def __del__(self):
        """
        Destructor of RobotInterface
        closes log file
        """

        self.__logFile.close()


    def getExternalEfforts(self):
        """
        getter-method for external efforts

        Returns:
            np.array[float64]: efforts [fx, fy, fz, mx, my, mz]
        """

        return self.__extEfforts
    

    def setUIP(self, marker, mbs):
        """
        method to set UIP in model and to add cllback function to apply effforts on it

        Args:
            marker (exudyn.exudynCPP.MarkerIndex): Exudyn marker at UIP position
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
        """

        # external applied forces
        def UFload(mbs, t, load):
            f = self.getExternalEfforts()[:3]
            f[0] = 0    # lock x direction
            return f
        
        mbs.AddLoad(LoadForceVector(markerNumber=marker,
                                    loadVector=[0, 0, 0],
                                    loadVectorUserFunction=UFload))

    
    def update(self, mbs, t):
        """
        method to be called once per frame/control cyle in Exudyn PreStepUserFunction
        reads sensor values, creates message out of them and sends corresponding order to robot
        publishes current system state for vr client

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
            t (float): elapsed time since simulation start

        Returns:
            exudyn.exudynCPP.MainSystem: modified mbs must be returned
        """

        # publishing each and every step is too much, this would slow down the connection
        # thus: publish every few seconds, only
        # furthermore, as vrInterface is only updating the graphics with f=60Hz, we don't have to update
        # system state every 1ms, so with f=1000Hz. Instead f=60Hz equivalents to update every 1/60=17ms

        if t - self.__lastRobotCommandSentTime >= self.__robotCommandSendInterval:
            # read current kinematic state and orientation
            pos_ = mbs.GetSensorValues(mbs.variables['pos'])
            vel_ = mbs.GetSensorValues(mbs.variables['vel'])
            acc_ = mbs.GetSensorValues(mbs.variables['acc'])

            #rot_ = mbs.GetSensorValues(mbs.variables['rotation'])
            
            # convert data to numpy arrays
            pos = np.array(pos_)
            vel = np.array(vel_)
            acc = np.array(acc_)
            #rot = np.array(rot_)

            # calculate angle (implemented but not used at the moment)
            #angleX = float(round(180+np.rad2deg(rot[0]), 4))
            angleX = 180.0  # robot EEF facing down

            self.__publishRobotCommand(pos, vel, acc, angleX, t)
            self.__lastRobotCommandSentTime = t

        if t - self.__lastSystemStateUpdateTime >= self.__systemStateUpdateInterval:
            # publish system state vor vrInterface
            # TODO belongs in __publishSystemState()
            systemStateData = mbs.systemData.GetSystemState()
            systemStateList1d = []

            # first entry is current time
            systemStateList1d.append(t)

            # then systemData itself follows
            for array in systemStateData:
                systemStateList1d.append(float(len(array)))
                for i in range(len(array)):
                    systemStateList1d.append(array[i])

            self.__publishSystemState(systemStateList1d)
            self.__lastSystemStateUpdateTime = t


    def simulate(self, mbs, simulationSettings):
        """
        Call exudyn solver for simulating system

        Args:
            mbs (exudyn.exudynCPP.MainSystem): multi-body simulation system from exudyn
            simulationSettings (exudyn.exudynCPP.SimulationSettings): Exudyn simulation settings
        """

        exu.SolveDynamic(mbs, simulationSettings)


    def __publishSystemState(self, systemData):
        """
        Publish current system state to ros-topic

        Args:
            systemData (list): full Exudyn SystemState concatenated in 1d list
        """

        msg = Float64MultiArray()
        
        msg.data = systemData # dataList
        self.__pubS.publish(msg)


    def __publishRobotCommand(self, posExu, vel, acc, angleX, tExu):
        """
        method to publish new robot command to ros-topic

        Args:
            posExu (np.array[float64], shape=(3,)): Cartesian position of user interaction point in Exudyn
            vel (np.array[float64], shape=(3,)): Cartesian velocity of user interaction point in Exudyn
            acc (np.array[float64], shape=(3,)): Cartesian acceleration of user interaction point in Exudyn
            angleX (float): rotation fo user interaction point around x-axis in Exudyn. Only used in impedance controller
            tExu (float): time since simulation start
        """

        # in first iteration, offset between exudyn local position and robot global position has to be determined
        if self.__firstPose:

            self.__posOffset = self.__globalStartPos - posExu
            self.__posOffset = np.expand_dims(self.__posOffset, axis=1)

            # rotation left aside for the moment
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
        callback method to set global robot position at beginning of program

        Args:
            data (geometry_msgs.msg.PoseStamped): current Cartesian pose of robot end-effector
        """

        if not self.__globalStartPosSet:
            self.__globalStartPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
            self.__globalStartPosSet = True
            # TODO Logger
            print("Global start pos set")


    def __externalEffortCallback(self, data):
        """
        callback method for external forces and torques, applied by the user on the robot
        for simplicity, forces and torques are denoted as efforts here

        every time new external efforts are registered by the robot(which happens continuously), 
        they are published and received by this callback function. Here they are written into 
        a global variable, in order to be processed by Exudyn later on

        Args:
            data (geometry_msgs.msg.WrenchStamped): registered force and torques
        """

        # for some reason, at the first few (3-4) calls of this callback-method, the frames panda_link0 is not known to tf
        # thus whole function is under try-statement, just to skip those first calls
        try:
            # transform forces and torques from K frame (endeffector, flange) (=panda_hand) into world frame (panda_link0)

            #                                            child          parent      use latest update
            (_,rot) = self.__tfListener.lookupTransform('/panda_link0', '/panda_K', rospy.Time(0))

            R = np.array(Rotation.from_quat(rot).as_matrix())

            # K frame
            forceK = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
            torqueK = np.array([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

            # world frame
            forceW = R @ forceK
            torqueW = R @ torqueK

            # as robot measures efforts it needs to compensate external force, negative efforts are the ones applied
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

                # additional to force offset, there is also some noise on force/torque measurement which has to be eliminated with threshold
                for i in range(6):
                    if abs(efforts[i]) > self.__effortsThreshold[i]:
                        self.__extEfforts[i] = efforts[i]
                    else:
                        self.__extEfforts[i] = 0

                # publish registered external force
                msgF = WrenchStamped()
                msgF.header.stamp = rospy.Time.now()
                msgF.wrench.force.x = self.__extEfforts[0]
                msgF.wrench.force.y = self.__extEfforts[1]
                msgF.wrench.force.z = self.__extEfforts[2]
                self.__pubF.publish(msgF)
        
        except Exception as e:
            pass
