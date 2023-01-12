#!/usr/bin/env python

"""
Author: Aaron Bacher
Date: 01.06.2022

Script for passing new target points to impedance controller
for testing purposes
"""
import sys

from matplotlib import use

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import tf
import math


# circle and square are centered around same point and have same outer dimensions
xc = 0.5
yc = 0.0
r = 0.15		# radius for circle, half side length of square

# for square contour calculation:
# we don't have access to current position of robot (else it would be easier to just change direction when certain position in x or y are reached) so we have to do it with time.
# Same as with circles, on which one rotation requires t = 2*pi, here one "round" on the square trajectory should take exaktly 2*pi
lTrajectory = 2*r*4		# length of full square-contour
squareStepSize = lTrajectory / (2*np.pi)	# size of one step in trajectory, in order to complete square after t = 2*pi


# calculate coordinates for circle
def trajectoryCircle(t):
	x = xc + r * np.cos(t)
	y = yc + r * np.sin(t)
	z = 0.2

	return (x, y, z)


# calculate coordinates for square
def trajectorySquare(t_):
	t = math.fmod(t_, 2*np.pi)	# time in current rotation

	if t <= 2*np.pi/8:
		x = xc + r
		y = yc + squareStepSize * t
	elif t <= 3*2*np.pi/8:
		x = xc + r - squareStepSize * (t-2*np.pi/8)
		y = yc + r
	elif t <= 5*2*np.pi/8:
		x = xc - r
		y = yc + r - squareStepSize * (t-3*2*np.pi/8)
	elif t <= 7*2*np.pi/8:
		x = xc - r + squareStepSize * (t-5*2*np.pi/8)
		y = yc - r
	else:
		x = xc + r
		y = yc - r + squareStepSize * (t-7*2*np.pi/8)

	z = 0.2

	return (x, y, z)


def trajectoryStepResponse(t_):
	t = math.fmod(t_, 10)

	if t <=5:
		x = 0.4
		y = 0.1
		z = 0.2
	else:
		x = 0.5
		y = 0.1
		z = 0.2

	return (x, y, z)


def main(usePoseController, trajectory, tPose=-1):

	# create correct publisher
	if usePoseController:
		pub = rospy.Publisher('/my_cartesian_pose_controller/setDesiredPose', PoseStamped, queue_size=10)
	else:
		#pub = rospy.Publisher('/my_cartesian_impedance_controller/setDesiredPose', PoseStamped, queue_size=10)
		pub = rospy.Publisher('/my_cartesian_impedance_controller/setTargetPose', PoseStamped, queue_size=10)

	# init rospy, init some variables
	rospy.init_node('newPoses', anonymous=True)
	rate = rospy.Rate(1000)
	t0 = rospy.Time.now()
	

	while not rospy.is_shutdown():
		# get time since program start (used as trajectory-parameter)
		if tPose != -1:	# go to one position only
			t = tPose
		else:	# move in circle
			t1 = rospy.Time.now()
			t = (t1-t0).to_sec()

		if trajectory == "square":
			coords = trajectorySquare(t)
		if trajectory == "circle":
			coords = trajectoryCircle(t)
		if trajectory == "stepResponse":
			coords = trajectoryStepResponse(t)

		# create message
		msg = PoseStamped()
		
		# write position into message
		msg.pose.position.x = coords[0]
		msg.pose.position.y = coords[1]
		msg.pose.position.z = coords[2]

		# endeffector should point straight down
		pitch = np.radians(180)
		roll = np.radians(0)
		yaw = np.radians(0)

		# create Quaternion out of Euler angles
		quaternion = tf.transformations.quaternion_from_euler(pitch, yaw, roll)

		# only to be sure quaternion is correct
		assert np.linalg.norm(quaternion) == 1.0, "ERROR"

		# write orientation into message
		msg.pose.orientation.x = quaternion[0]
		msg.pose.orientation.y = quaternion[1]
		msg.pose.orientation.z = quaternion[2]
		msg.pose.orientation.w = quaternion[3]

		# write current time into message
		msg.header.stamp = rospy.Time.now()
		#print(msg)
		pub.publish(msg)

		rate.sleep()


if __name__ == '__main__':
	try:

		# default values
		usePoseController = False
		trajectory = "circle"
		tPose = -1

		# parse arguments
		i = 0
		while i < len(sys.argv):
			arg = sys.argv[i]
			if i == 0:
				pass	# program name is no for us relevant parameter

			elif arg == "-i" or arg == "-I":
				pass	# nothing to do, as usePoseController is alredy false and thus impedance control active
			elif arg == "-p" or arg == "-P":
				usePoseController = True

			elif arg == "-c" or arg == "-C":
				pass	# nothing to do, as trajetory is alredy set to circle
			elif arg == "-s" or arg == "-S":
				trajectory = "square"
			elif arg == "-sR" or arg == "-sr" or arg == "-SR" or arg == "-Sr":
				trajectory = "stepResponse"

			elif arg == "-t":
				if len(sys.argv) > i+1:
					try:
						tPose = float(sys.argv[i+1])
						i += 1	# skip next iteration
					except ValueError:
						print("Argument -t requires float as parameter")
						exit()
				else:
					print("Argument -t requires float as parameter")
					exit()
			else:
				print("Unknown argument: " + str(arg))
				exit()
			
			i += 1

		# inform user
		print("Sending ", end="")
		if tPose != -1:
			print("t={} position of ".format(tPose), end="")
		print(trajectory, end="")
		print("-trajectory to ", end="")
		if usePoseController:
			print("pose", end="")
		else:
			print("impedance", end="")
		print("-control...")
		print("Terminate movement with Ctrl+C")

		# execute
		main(usePoseController, trajectory, tPose)

	except rospy.ROSInterruptException:
		pass
