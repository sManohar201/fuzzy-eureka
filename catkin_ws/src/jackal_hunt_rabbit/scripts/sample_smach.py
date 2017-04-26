#!/usr/bin/env python

#----------------------------------------------------------
# FINAL PROJECT - EE5900 Intro to Robotics
# State machine implementation for the project
# Team Members
# 	1) Deep Doshi (Team Lead)
#  	2) Haden Wasserbaech
# 	3) Sabari Manohar
#----------------------------------------------------------

# load the dependencies for the project
from smach import StateMachine, State, Concurrence
import smach_ros #### donot know why it is loaded
import rospy
# This is the message type the robot uses for velocities
from geometry_msgs.msg import Twist 
# This is the message type the robot uses for laserscan
from sensor_msgs.msg import LaserScan
# numpy library for array manipulation
import numpy as np
import roslib 


""" EXPLORER -- LASERSCAN """
# Subscriber callback function
def get_range(dt):
	global left_range, right_range, middle_range

def run():
	return 0.3


class Explorer(State):
	"""Exploration of the area."""
	def __init__(self):
		State.__init__(self, outcomes=['ExploredArea'])

	def execute(self, userdata):
		# log starting of exploration
		rospy.loginfo("Exploration Started !")

		left_range, right_range, middle_range = 0 #### needs change
		# update rate of 10 Hz
		rate = rospy.Rate(10)
		# subscribe to laser data and calls get_range function
		rospy.Subscriber("/scan", LaserScan, get_range)
		pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
		
		#### TRY TO GET THE OUTCOME OF THE ALVAR DETECTOR
		for i in range(300):
			twist_msg = Twist()
			twist_msg.angular.z = run()
			pub.publish(twist_msg)
			pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=5)
			rate.sleep()

		return 'finish'

class Alvar(State):
	"""Alvar detection while exploration."""
	def __init__(self):
		State.__init__(self, outcomes=['AlvarTracked'])

	def execute(self, userdata):
		rospy.loginfo()

class Rotate_anti(State):
	def __init__(self):
		State.__init__(self, outcomes=['finish'])
		
		
	def execute(self, userdata):
		pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
		rate = rospy.Rate(10)
		for i in range(300):
			twist_msg = Twist()
			twist_msg.angular.z = -run()
			pub.publish(twist_msg)
			pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=5)
			rate.sleep()

		return 'finish'


def main():
	rospy.init_node('StateMachine')

	sm = Concurrence(outcomes=['FinalSuccess'])
	
	with sm:

		StateMachine.add('Clock', Explorer(), transitions={'finish':'AntiClock'})
		StateMachine.add('AntiClock', Rotate_anti(), transitions={'finish':'FinalSuccess'})


	sm.execute()

if __name__ == '__main__':
	main()