#!/usr/bin/env python

# joystick_controlstart.py
# Use joystick input to launch object tracking nodes in jackal
# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #6

#       Project #6 Group #4
#            Haden
#            Deep (Teamlead)
#            Sabari 
#
# /blueooth_teleop/joy
# sensor_msgs/Joy
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32[] axes
# int32[] buttons
#
# axes: [ lft - l/r, lft - up/down, L2 (1/-1), rgt - l/r, rgt - u/d, R2 (1/-1)]
# buttons: [ x, circle, sq, tri, L1, R1, share, options, play, L3, R3, DL, DR, DU, DD]
#

import rospy
import roslaunch
import sys
import time
import os
# import track
from sensor_msgs.msg import Joy

class joy_control(object):

    def __init__(self):

        rate = rospy.Rate(5)
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)
        # Initialize button variables for button input
        self.x = 0
        self.circ = 0
	self.sq = 0
        self.tri = 0
 	self.L1 = self.R1 = self.share = self.options = self.p4 = self.L3 = self.R3 = self.DL = self.DR = self.DU = self.DD = 0
        # Boolean variable used for tracking the status of object tracking process
        self.active1 = False
        self.active2 = False
	self.active3 = False
        objtrack_process = None
	
        rospy.loginfo("In start")

        while not rospy.is_shutdown():

            # If object tracking process is not currenlty active
            if self.active1 == False or self.active2 == False or self.active3 == False:

                # Start Object Tracking if circle button pressed
                if (self.circ == 1):
                    rospy.loginfo("Joystick code received, commencing object tracking protocol...")
                    self.active = True

                    package = 'jackal_hunt_rabbit'
                    executable = 'explore.py'
                    node = roslaunch.core.Node(package, executable)
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    objtrack_process = launch.launch(node)

		if (self.sq == 1):
		    rospy.loginfo("Joystick code received, commencing  protocol...")
		    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		    roslaunch.configure_logging(uuid)
		    path = rospkg.RosPack()
		    r_path = str(path.get_path("jackal_hunt_rabbit")) + "/launch/gmapping_demo.launch"
		    launch = roslaunch.parent.ROSLaunchParent(uuid, [r_path])

		    launch.start()
	
                    objtrack_process = launch.launch(node)

            # If object tracking process is active
            else:
                # Stop Object Tracking if x button pressed
                if (self.x == 1):
                    rospy.loginfo("Joystick code recieved, terminating object tracking protocol")
                    self.active = False
                    objtrack_process.stop()
            # Reset button variables for next pass
            self.x = 0
            self.circ = 0
	    self.sq = 0

    # callback function maps button data observed from joystick topic
    def joy_callback(self, data):
        self.x, self.circ, self.sq, self.tri, self.L1, self.R1, self.share, self.options, self.p4, self.L3, self.R3, self.DL, self.DR, self.DU, self.DD = data.buttons


if __name__ == "__main__":

    try:
        rospy.init_node("joy_controlstart", anonymous=False)
        run = joy_control()  #read in joystick input
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
