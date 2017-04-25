#!/usr/bin/env python

import smach
import smach_ros
import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities
from sensor_msgs.msg import LaserScan

import roslib 


class Rotate_clock(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finish'])
		rospy.loginfo("rotate clockwise")

	def execute(self, userdata):
		pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
		rate = rospy.Rate(10)
		for i in range(300):
			twist_msg = Twist()
			twist_msg.angular.z = 0.3
			pub.publish(twist_msg)
			pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=5)
			rate.sleep()

		return 'finish'

class Rotate_anti(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finish'])
		rospy.loginfo("rotate anti clockwise")
		
	def execute(self, userdata):
		pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
		rate = rospy.Rate(10)
		for i in range(300):
			twist_msg = Twist()
			twist_msg.angular.z = -0.3
			pub.publish(twist_msg)
			pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=5)
			rate.sleep()

		return 'finish'


def main():
	rospy.init_node('StateMachine')

	sm = smach.StateMachine(outcomes=['success'])
	
	with sm:

		smach.StateMachine.add('Clock', Rotate_clock(), transitions={'finish':'AntiClock'})
		smach.StateMachine.add('AntiClock', Rotate_anti(), transitions={'finish':'success'})


	sm.execute()

if __name__ == '__main__':
	main()