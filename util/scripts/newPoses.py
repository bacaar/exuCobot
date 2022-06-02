#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose, Vector3

import numpy as np
import tf

import time

def talker():

    pose = True
    pub = None
    if pose:
        pub = rospy.Publisher('/my_cartesian_impedance_example_controller/setDesiredPose', Pose, queue_size=10)
    else:
        pub = rospy.Publisher('/my_cartesian_impedance_example_controller/setDesiredPosition', Vector3, queue_size=10)

    rospy.init_node('newPoses', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    npos = 1
    npose = 1

    #t1 = time.time()
    rising = True
    x = 0.4

    while not rospy.is_shutdown():
        #t2 = time.time()
        #dt = t1-t2
        #print(dt)

        if rising:
            x = x + 0.01
        else:
            x = x - 0.01

        if x > 0.7:
            rising = False
        if x < 0.4:
            rising = True

        if pose:
            msg = Pose()
            msg.position.x = x
            msg.position.y = 0.1
            msg.position.z = 0.4
            # Make sure the quaternion is valid and normalized

            # 90 degree rotation around one axis
            pitch = np.radians(180)
            roll = np.radians(0)
            yaw = np.radians(0)
            quaternion = tf.transformations.quaternion_from_euler(pitch, yaw, roll)
            assert np.linalg.norm(quaternion) == 1.0, "ERROR"

            msg.orientation.x = quaternion[0]
            msg.orientation.y = quaternion[1]
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]

            print("publish Pose " + str(npose))
            npose = npose+1

        else:
            msg = Vector3()
            msg.x = 0.4
            msg.y = -0.1
            msg.z = 0.6

            print("publish Position " + str(npos))
            npos = npos+1

        print(msg)
        pub.publish(msg)

        #t2 = time.time()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
