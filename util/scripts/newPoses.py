#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose, Vector3

def talker():

    pose = False
    pub = None
    if pose:
        pub = rospy.Publisher('/my_cartesian_impedance_example_controller/setDesiredPose', Pose, queue_size=10)
    else:
        pub = rospy.Publisher('/my_cartesian_impedance_example_controller/setDesiredPose', Vector3, queue_size=10)

    rospy.init_node('newPoses', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 1

    while not rospy.is_shutdown():
        if pose:
            msg = Pose()
            msg.position.x = 0.5
            msg.position.y = -0.1
            msg.position.z = 0.6
            # Make sure the quaternion is valid and normalized
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            pub.publish(msg)
        else:
            msg = Vector3()
            msg.x = 0.5
            msg.y = -0.1
            msg.z = 0.6
            pub.publish(msg)

        print("published " + str(i))
        i = i+1

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
