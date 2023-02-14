"""
Author: Aaron Bacher
Date: 14.02.2023

Combined Robot and VR Interface for Exudyn
"""

from VrInterface import VrInterface
from RobotInterface import RobotInterface

class RobotVrInterface:

    # TODO: what if interfaceType==2 but robotInterface not available?
    def __init__(self, interfaceType, useImpedanceController=False) -> None:
        """
        Initialization

        :param int interfaceType: may be 1 if this instance should be the robot interface, 2 for the vr interface
        :param bool useImpedanceController: must be true if running robot controller is impedance controller. Default false for velocity controller
        """
        
        assert interfaceType == 1 or interfaceType == 2, "Undefined interface type"

        if interfaceType == 1:
            self.__robotInterface = RobotInterface(useImpedanceController)
        else:
            self.__vrInterface = VrInterface(useImpedanceController)

    def update(self, mbs, t) -> None:
        pass