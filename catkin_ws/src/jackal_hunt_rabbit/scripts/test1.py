#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
	global closest_range
	global maxi_range
	global min_range
	global min_angle
	global max_angle
	global update_frequency
	global angle_incre
	global angle_time
	global vel_x
	global length
	global length1

	closest_range = min(msg.ranges)
	length = len(msg.ranges)
	length1 = (msg.angle_max-msg.angle_min)/msg.angle_increment
	"""maxi_range = msg.range_max
	min_range = msg.range_min
	min_angle = msg.angle_min
	max_angle = msg.angle_max
	
	update_frequency = msg.scan_time
	angle_incre = msg.angle_increment
	angle_time = msg.time_increment"""
	vel_x = msg.linear.x

	print("Got values!")

if __name__ == "__main__":
	closest_range = 1.0
	length = 0
	length1 = 0
	try:
		rospy.init_node("Testing")
		rospy.Subscriber("/scan", LaserScan, callback)
		rate = rospy.Rate(50)
		twist = Twist()
		lasermsg = LaserScan()
		while not rospy.is_shutdown():
			print("Closest Range : " + str(closest_range))
			print("total length : " + str(length))
			print(str(length1))
	except rospy.ROSInterruptException:
		pass
