import roboticstoolbox as rtb

import numpy as np

import rospy
from spatialmath import SE3

robot = rtb.models.DH.Panda()

def fkin():

    # initial joint angles, approx the pose we want
    q0 = [0.0, -0.22449781786768058, 0.0, -2.6762069058915854, 0.0, 2.4217752625883384, 0.7664495172173067]

    # print pos and orientation
    print("q0")
    print(q0)
    print(robot.fkine(q0))

    # set precise pose we want
    T = SE3.Rt(np.eye(3), np.array([0.4, 0, 0.2]))@SE3.RPY(np.pi, 0, 0)
    print("we want")
    print(T)

    # calculate needed joint angles for precise pose
    q1 = robot.ikine_LM(T, q0=q0).q
    print("q1")
    print(q1)
    print(robot.fkine(q1))

    # calculated angles are not so pretty -> modify them
    q2 = q1
    q2[0] = 0
    q2[2] = 0
    q2[4] = 0
    print("q2")
    print(q2)
    print(robot.fkine(q2))






if __name__ == "__main__":
    fkin()
