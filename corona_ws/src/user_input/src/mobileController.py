#!/usr/bin/env python

# 2.12/2.120 Final Project
# Best Team

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

import serial
from apriltag_ros.msg import AprilTagDetectionArray
from user_input.msg import JoyCmd

import math

# Left-Right distance between camera and tag
deltaX = 0.01
# Depth distance between camera and tag
deltaZ = 0.01
# Orientation of tag relative to camera
tag_angle = 0.01

# Camera velocities
Vx = 0 # Positive goes right
Vz = 0 # Positive goes forward
Vtheta = 0 # Positive rotates left (assumed?)


# X offset is the X distance between the tag and our desired final position
	# Positive is to the right of the tag
# Z offset is the Z distance between the tag and our desired final position
	# Positive is in front of the tag
# Theta offset is the angle we want the tag to be relative to us
	# If 0, then we want to be perpendicular to the tag
# X and Z offsets are in meters
# Offsets are for [ tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_5, tag_4, tag_3 ]
X_offset =        [     0,     0,     0,     0,     0,     0,     0,     0,     0 ]
Z_offset =        [  0.35,  0.35,  0.35,  0.35,  0.35,  0.35,   1.4,   3.7,   2.5 ]
Theta_offset =    [     0,     0,     0,     0,     0,     0,     0,     0,     0 ]

# Largest ID the camera can currently see
largest_ID = 0

# True if the keyboard is inputting manual control
# Stops this file from publishing the velocities
manualControl = False

# Callback function for the AprilTag detection subscriber
# Finds the largest ID AprilTag in view
# Stores X, Z distance and orientation of tag
def viewedTagRelPos(data):

	global deltaX, deltaZ, largest_ID, tag_angle

	# Find all IDs in view
	IDs_in_view = []
	for detection in data.detections:
		IDs_in_view.append( detection.id[0] )
	
	# largest_ID is -1 if no IDs are in view
	if len( IDs_in_view ) == 0:
		print( "No ID in view" )
		largest_ID = -1
	# Otherwise, set deltaX, deltaZ, largest_ID, and tag_angle
	else:
		largest_ID = max( IDs_in_view )
		for detection in data.detections:
			if( detection.id[0] == largest_ID ):
				print( "Largest ID:", largest_ID )
				tag_ID = largest_ID

				deltaX = detection.pose.pose.pose.position.x
				deltaZ = detection.pose.pose.pose.position.z
				print( "deltaX =", deltaX, "deltaZ =", deltaZ)

				quatX = detection.pose.pose.pose.orientation.x
				quatZ = detection.pose.pose.pose.orientation.z

				tag_angle = math.atan( quatZ / quatX )
				print( "Tag Angle:", tag_angle)

# Rotate without any linear velocity until targetTagID is in center of view
# i.e. angle between our vision and center of tag < 0.05 radians
def pointAtTag(targetTagID, direction = 1):

	global Vx, Vz, Vtheta

	# No linear velocity, rotate only
	Vx = 0
	Vz = 0

	# If we can't see the targetTag, rotate in direction
	while largest_ID != targetTagID:
		print("No tag in view")
		Vtheta = direction*0.05
		publishVelocities()

	# If angle between us and tag is > 0.05 radians, rotate towards it
	# Increase speed proportionally to angle
	while abs(math.atan(deltaX/deltaZ)) > 0.05:

		print("Not pointed at tag")
		Vtheta = -1*math.atan(deltaX/deltaZ)
		publishVelocities()

	print("Pointed at tag")


# Move towards tag with prescribed X, Z, and angular offset
# X offset is the X distance between the tag and our desired final position
	# Positive is to the right of the tag
# Z offset is the Z distance between the tag and our desired final position
	# Positive is in front of the tag
# Theta offset is the angle we want the tag to be relative to us
	# If 0, then we want to be perpendicular to the tag
def approachTag(targetTagID, offsetID):
	tag_X_offset = X_offset[ offsetID ]
	tag_Z_offset = Z_offset[ offsetID ]
	tag_angle_offset = Theta_offset[ offsetID ]

	global Vx, Vz, Vtheta

	# Get target position and angle based on desired offsets and current position
	target_X = deltaX + tag_X_offset*math.cos(tag_angle) + tag_Z_offset*math.sin(tag_angle)
	target_Z = deltaZ + tag_X_offset*math.sin(tag_angle) - tag_Z_offset*math.cos(tag_angle)
	target_theta = tag_angle - tag_angle_offset

	# If we can't see the tag, point at the tag again
	if( largest_ID != targetTagID ):
		print("Target tag not in view")
		pointAtTag(targetTagID)

	# Keep running this loop until we are at our target position and orientation
	while abs(target_X) > 0.05 or abs(target_Z) > 0.05 or abs(target_theta) > 0.05:

		# If we can't see the tag, point at the tag again
		if( largest_ID != targetTagID ):
			print("Lost tag")
			pointAtTag(targetTagID)
		# Otherwise, recalculate our target position and orientation
		# Calculate velocities to reach target
		else:
			target_X = deltaX + tag_X_offset*math.cos(tag_angle) + tag_Z_offset*math.sin(tag_angle)
			target_Z = deltaZ + tag_X_offset*math.sin(tag_angle) - tag_Z_offset*math.cos(tag_angle)
			target_theta = tag_angle - tag_angle_offset

			# Velocity calculated as unit vector in direction of target
			Vx = target_X / math.sqrt( target_X**2 + target_Z**2 )
			Vz = target_Z / math.sqrt( target_X**2 + target_Z**2 )

			# If our target orientation is not right, rotate towards the correct orientation
			# If tag is at a positive angle relative to us, rotate left
			# If tag is at a negative angle relative to us, rotate right
			if abs(target_theta) > 0.05:
				Vtheta = 0.05*target_theta
				print("Rotate towards tag")
			# If we are pointing the right direction, only move with linear velocity
			else:
				Vtheta = 0
				print("Aligned with tag")

		publishVelocities() 

	print("Arrived at tag")

# Publishes the current Vx, Vz, Vtheta values to /joy/cmd
def publishVelocities():

	# If the keyboard is being used for manual control, do not publish velocities
	if manualControl:
		print( "Manual Control" )
	# Otherwise, publish, Vx, Vz, Vtheta
	else:
		jcv = JoyCmd()

		jcv.axis1 = Vx
		jcv.axis2 = Vz
		jcv.axis3 = Vtheta

		jcv.btn1 = 0.0
		jcv.btn2 = 0.0
		jcv.btn3 = 0.0

		print("Vx =", Vx, "Vz =", Vz, "Vtheta =", Vtheta)

		virtualJoy_pub.publish( jcv )

# Callback function for determining if the keyboard is being used to control the robot
# If /manual_control topic becomes True, stops the robot
def manualControlCallback(data):
	global manualControl
	# True when manual control is happening
	# False otherwise
	manualControl = data.data

	# If True, stop the robot and stop allowing this file to publish to /joy/cmd
	if manualControl:
		jcv = JoyCmd()

		jcv.axis1 = 0
		jcv.axis2 = 0
		jcv.axis3 = 0

		jcv.btn1 = 0.0
		jcv.btn2 = 0.0
		jcv.btn3 = 0.0

		virtualJoy_pub.publish( jcv )

# Publish the location of the basket to /basket_location
# TODO: Make this work how we need it do
def publishBasketLocation():
	pnt = Point()

	pnt.x = deltaX
	pnt.y = 0
	pnt.z = deltaZ

	basket_pub.publish( pnt )
	
# Subscribes to /tag_detections
# Passes the message (AprilTagDetectionArray) into function viewedTagRelPos(data)
apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, viewedTagRelPos)
print("AprilTag Subscriber setup")

# Allow publishing of velocity commands to /joy/cmd
virtualJoy_pub = rospy.Publisher("/joy/cmd", JoyCmd, queue_size=1)
print("Joy Command Publisher setup")

# Subscribe to topic that is True when the robot is under manual control
# False when not under manual control
# Publish to this topic from the keyboard_process.py node
# In publishVelocities(), only publish the velocities from this node if topic is False
manual_control_sub = rospy.Subscriber("/manual_control", Bool, manualControlCallback)
print("Manual Control Subscriber setup")

# Allow publishing of basket location to /basket_location
basket_pub = rospy.Publisher("/basket_location", Point, queue_size=1)
print("Basket Location Publisher setup")

# Random ros stuff
rospy.init_node("controller")
rospy.Rate(50)
print("ROS rate setup")

while not rospy.is_shutdown():
	print("Start loop")
	# Look for first tag
	pointAtTag(1)
	# Move towards first tag
	approachTag(1,1)

	# After we find the first tag, rotate right to find second tag
	pointAtTag(2, direction = -1)
	approachTag(2,2)

	# Rotate right to find third tag
	pointAtTag(3, direction = -1)
	approachTag(3,3)

	# Rotate left to find fourth tag
	pointAtTag(4, direction = 1)
	approachTag(4,4)

	# Rotate left to find fifth tag
	pointAtTag(5, direction = 1)
	approachTag(5,5)

	# Stop robot
	jcv.axis1 = 0.0
	jcv.axis2 = 0.0
	jcv.axis3 = 0.0
	jcv.btn1 = 0.0
	jcv.btn2 = 0.0
	jcv.btn3 = 0.0
	virtualJoy_pub.publish(jcv)

	# Once docked, publish basket location
	publishBasketLocation()
	
	#TODO figure out when UR team is done
	
	# Rotate left to find fifth tag, then back away from it
	pointAtTag(5, direction = 1)
	approachTag(5, 6)
	
	# Rotate right to find fourth tag and then back away
	pointAtTag(4, direction = -1)
	approachTag(4, 7)
	
	# Rotate right to find third tag and then back away
	pointAtTag(3, direction = -1)
	approachTag(3, 8)
	
	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		r.sleep()
