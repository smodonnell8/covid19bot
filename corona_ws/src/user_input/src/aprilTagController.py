#! /usr/bin/env python

# 2.12 Final Project
# Phillip Daniel April 2021

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

import serial
from apriltag_ros.msg import AprilTagDetectionArray
from user_input.msg import Velocity, JoyCmd

# These global variables are the pose of the mobile robot
x= 0.0
y=0.0
theta=0.0
tagPose = None
tagID = None

def pointAtTag(targetTagID):
	# Description: Search for the specified tag
	# Return
		# void - Prints a message when we are pointing at a tag
	# Argument
		# targetTagID - The ID of the tag that we wish to point the robot's camera towards

	zDot=0
	xDot=0
	rate=.75
	pointed = False
	
	global tagPose
	global tagID
	
	jcv = JoyCmd()
	jcv.axis1 = 0.0
	jcv.axis2 = 0.0
	jcv.btn1 = 0.0
	jcv.btn2 = 0.0
	jcv.btn3 = 0.0


	while pointed == False:
		
		# Spin around and look for the tag 'locatedTag.label.' Stop once we are pointing at it within a small window for error
		if targetTagID == tagID: # We have found the tag that we were looking for.
			relX=tagPose.pose.pose.position.x
			if relX<.05 and relX>-.05: # If we are pointing at the tag
				thetaDot=0
				pointed = True
			else:
				pointed = False
		else:
			thetaDot=rate
			pointed = False

		jcv.axis3 = thetaDot
		virtualJoy_pub.publish(jcv)
		r.sleep()
	print('Pointed at tag')

def viewedTagRelPos(data):
	# Description: Relative position of the tag 'tagID,' if it is in view
		# Return
			# locatedTag.label - The label for the visible April tags (e.g. 'tag1')
			# locatedTag.relZ - The x-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
			# locatedTag.relX - The y-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
		# Argument
			# tagID - The ID of the tag that we wish to point the robot's camera towards

	global tagPose
	global tagID
	print('Check for pose of tags')
	tagPose = None
	tagID = None
	for detections in data.detections:
		print detections.id
		for idseen in detections.id:
			if idseen == 0 or idseen == 1 or idseen == 2 or idseen == 3 or idseen == 4 or idseen == 5 :
				tagID = idseen
				tagPose = detections.pose
				print('tag', tagID, 'detected')

def approach():
	# Description: Drive within a certian distace of the tag 'targetTagID'
		# Return
			# null
		# Argument
			# targetTagID - The ID of the tag that we wish to point the robot's camera towards
	global tagPose
	relX=tagPose.pose.pose.position.x
	relZ=tagPose.pose.pose.position.z
	approached=False
	
	jcv = JoyCmd()
	jcv.axis3 = 0.0
	jcv.btn1 = 0.0
	jcv.btn2 = 0.0
	jcv.btn3 = 0.0
	
	while approached==False:
		relX=tagPose.pose.pose.position.x
		relZ=tagPose.pose.pose.position.z
		
		vRel=np.array([relZ,relX])
		relPosNorm=np.linalg.norm(vRel)
		relPosUnitVec=vRel/relPosNorm
		thetaDot=0

		if relPosNorm > .2:
			zDot=relPosUnitVec[0]
			xDot=relPosUnitVec[1]
			approached=False
		else:
			zDot=0
			xDot=0
			approached=True

		jcv.axis1 = xDot
		jcv.axis2 = zDot
		virtualJoy_pub.publish(jcv)
		r.sleep()
	print('Arrived at tag')
	
	
rospy.init_node('TagChaser', anonymous=True)

apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, viewedTagRelPos)
virtualJoy_pub = rospy.Publisher("/joy/cmd", JoyCmd)

speed = Twist()

r = rospy.Rate(100)


# This is the main loop
while not rospy.is_shutdown():
	pointAtTag(0)
	approach()
	
	while 1:
		r.sleep()
