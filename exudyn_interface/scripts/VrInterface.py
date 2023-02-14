"""
Author: Aaron Bacher
Date: 07.02.2023

VR Interface for Exudyn
"""

import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray

class VrInterface:

    def __init__(self, useImpedanceController) -> None:

        self.__impedanceController = useImpedanceController
        self.__globalStartPos = np.array([0.0, 0.0, 0.0])
        self.__globalStartPosSet = False

        self.__systemStateData = None

        rospy.init_node('ExudynVrInterface', anonymous=True)
        # subscriber for current pose. Needed for localizing current end-effector position in vr-space
        if self.__impedanceController:
            topicBase = "/my_cartesian_impedance_controller"
        else:
            topicBase = "/my_cartesian_velocity_controller"

        self.__systemStateSub = rospy.Subscriber(topicBase + "/SystemState", Float64MultiArray, self.__systemStateCallback)
        globalStartPosSub = rospy.Subscriber(topicBase + "/getCurrentPose", PoseStamped, self.__currentPoseCallback)

        # TODO: check if controller available
        # TODO: if yes, set origin relative to controller and robot configuration
        # TODO: check if tracker (for hand rendering) available
        pass

    def __del__(self):
        #self.__systemStateSub.unregister()
        pass

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


    def getCurrentSystemState(self):
        return self.__systemStateData


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