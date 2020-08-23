#!/usr/bin/python


import rospy
import numpy as np
from numpy.linalg import inv

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32
from user_input.msg import Velocity, JoyCmd
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

class joybrain(object):

    def __init__(self):
        rospy.init_node('joybrain', anonymous=True)
        rospy.Subscriber("/joy/connected", Bool, self.joyconnection_cb)
        rospy.Subscriber("/joy/cmd", JoyCmd, self.joycmd_cb)
        self.connected = False
        self.browser_pub = rospy.Publisher('/cmd2browser', Int32, queue_size = 1)
        self.cmdVel1_pub = rospy.Publisher('/robot/velocity/cmd1', Velocity, queue_size = 10)
        self.cmdVel2_pub = rospy.Publisher('/robot/velocity/cmd2', Velocity, queue_size = 10)

        r = rospy.Rate(60)
        self.vel1_msg = Velocity()
        self.vel2_msg = Velocity()
        self.browser_msg = Int32()
        self.browser_msg.data = 0

        while not rospy.is_shutdown():
            # Publish at a frequency of 60 Hz
            self.browser_pub.publish(self.browser_msg)
            self.cmdVel1_pub.publish(self.vel1_msg)
            self.cmdVel2_pub.publish(self.vel2_msg)
            r.sleep()

    def joyconnection_cb(self, msg):
        self.connected = msg.data

    def joycmd_cb(self, msg):
        print("axis1: " + str(msg.axis1))

if __name__=='__main__':
    node = joybrain()
    
