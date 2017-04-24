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
	global length
	global update_frequency
	global angle_incre
	global angle_time
	global vel_x

	closest_range = min(msg.ranges)
	"""maxi_range = msg.range_max
	min_range = msg.range_min
	min_angle = msg.angle_min
	max_angle = msg.angle_max
	length = len(msg.ranges)
	update_frequency = msg.scan_time
	angle_incre = msg.angle_increment
	angle_time = msg.time_increment"""
	vel_x = msg.linear.x

	print("Got values!")

if __name__ == "__main__":
	closest_range = 1.0
	vel_x = 0
	try:
		rospy.init_node("Testing")
		rospy.Subscriber("/scan", LaserScan, callback)
		rate = rospy.Rate(50)
		twist = Twist()
		lasermsg = LaserScan()
		while not rospy.is_shutdown():
			print(closest_range)
	except rospy.ROSInterruptException:
		pass
