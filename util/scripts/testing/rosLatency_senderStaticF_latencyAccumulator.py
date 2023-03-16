"""
Author: Aaron Bacher
Date: 14.02.2023
Sender script. Sends message and waits for response
sending frequency is static, analyzes how latency gets influenced by ongoing sending/receiving
"""

import rospy
from util.msg import segmentCommandStamped

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import sys

receivedData = []
dataAcquisitionEnabled = False

def myCallback(msg):
    if dataAcquisitionEnabled:
        global receivedData
        receivedData.append([msg, rospy.Time.now()])
        #receivedData.append([msg, 0])

"""
    freq = sending frequency
    nSamples = n samples to send
"""
def main(freq, nSamples):

    print("Analyzing f=" + str(freq) + "Hz for " + str(nSamples/freq) + "s")

    # init ROS    
    rospy.init_node("Sender")

    # setup publisher for sending new messages
    pub = rospy.Publisher('/rosLatencyAnalyzer/sender', segmentCommandStamped, queue_size=1000)

    # setup subsciber for getting the answer
    rospy.Subscriber("/rosLatencyAnalyzer/responder", segmentCommandStamped, myCallback)

    # create a message
    msg = segmentCommandStamped()
    msg.header.seq = 0

    global dataAcquisitionEnabled
    dataAcquisitionEnabled = True

    rate = rospy.Rate(freq)

    # send messages
    for i in range(nSamples):
        
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        
        rate.sleep()

    # sleep for five seconds just to get all remaining answers
    rospy.sleep(5)
    dataAcquisitionEnabled = False

    receivedMsgs = np.zeros(shape=(len(receivedData), 2))

    # evaluate received answers
    for i, data in enumerate(receivedData):
        timeSent = rospy.Time(data[0].header.stamp.secs, data[0].header.stamp.nsecs)

        buf = data[0].header.frame_id

        s = buf.split('_')[0]
        ns = buf.split('_')[1]
        timeSentBack = rospy.Time(int(s), int(ns))
        timeReceived = data[1]

        dt1 = (timeSentBack - timeSent).to_sec()

        #if dt1 < 0:
        #    print(dt1)

        dt2 = (timeReceived - timeSentBack).to_sec()
        #dt2 = 0

        receivedMsgs[i] = np.array([dt1, dt2])

    df = pd.DataFrame(receivedMsgs, columns = ['dt1','dt2'])
    fileName = "f" + str(freq).zfill(4) + "_ns" + str(nSamples) + "_nr" + str(len(receivedData)) + ".csv"
    df.to_csv("accumulatedLatencies/" + fileName)

    #fig, ax = plt.subplots(1,1)
    #ax.plot(receivedMsgs[:,0])
    #ax.plot(receivedMsgs[:,1])
    #plt.show()


if __name__ == "__main__":
    
    if len(sys.argv) == 3:
        f = int(sys.argv[1])
        n = int(sys.argv[2])
        main(f, n)
    elif len(sys.argv) == 2:
        f = int(sys.argv[1])
        n = f * 20
        main(f, n)
    else:
        print("Usage: python3 senderStaticF.py [frequency] ([nSamples])")