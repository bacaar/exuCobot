"""
Author: Aaron Bacher
Date: 07.02.2023

VR Interface for Exudyn
"""

import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped, WrenchStamped

class VrInterface:

    def __init__(self, useImpedanceController) -> None:

        self.__impedanceController = useImpedanceController
        self.__globalStartPos = np.array([0.0, 0.0, 0.0])
        self.__globalStartPosSet = False

        rospy.init_node('ExudynVrInterface', anonymous=True)

        # subscriber for current pose. Needed for localizing current end-effector position in vr-space
        if self.__impedanceController:
            globalStartPosSub = rospy.Subscriber("/my_cartesian_impedance_controller/getCurrentPose", PoseStamped, self.__currentPoseCallback)
        else:
            globalStartPosSub = rospy.Subscriber("/my_cartesian_velocity_controller/getCurrentPose", PoseStamped, self.__currentPoseCallback)

        # TODO: check if controller available
        # TODO: if yes, set origin relative to controller and robot configuration
        # TODO: check if tracker (for hand rendering) available
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
        SC.visualizationSettings.general.graphicsUpdateInterval = 0.005


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