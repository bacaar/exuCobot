"""
Author: Aaron Bacher
Date: 08.02.2023
Responder script. Waits for message and sends it back
"""

import rospy
from util.msg import segmentCommandStamped

gotAnswer = False
pub = None

def myCallback(data):
    time = rospy.Time.now()
    data.header.frame_id += str(time.secs) + "_" + str(time.nsecs)
    pub.publish(data)


def main():

    # init ROS    
    rospy.init_node("Responder")

    global pub
    # setup publisher for sending new messages
    pub = rospy.Publisher('/rosLatencyAnalyzer/responder', segmentCommandStamped, queue_size=1000)

    # setup subsciber for getting the answer
    rospy.Subscriber("/rosLatencyAnalyzer/sender", segmentCommandStamped, myCallback)

    while not rospy.is_shutdown():
        rospy.sleep(-1)
        #rospy.spin()



if __name__ == "__main__":
    main()