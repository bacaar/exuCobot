# Example program

Directory containes Exudyn example scripts simulating single pendulum with UIP at pendulum tip. All python scripts named *example<...>.py* are intermediate stages that have been saved during developement as additional version control, to be able to access older versions easier. Final version is *example4_1.py*.

# Interface
Directory also contains the developed interface, that is used in later development stages of the above mentioned scripts. This is the *RobotVrInterface.py*. The *RobotInterface.py* and the *VrInterface.py* again are previous developement stages.

**Important**: As there is a .stl file used in *RobotVrInterface.py*, to which the path is still hardcoded, the scripts must be executed from within this directory. The .stl file is located in the *hand_stl/* directory and was taken from [here](https://grabcad.com/library/articulated-dummy-1).
