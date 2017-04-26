#!/usr/bin/env python

# Code heavily borrowed from Project #3, Group #2 semi-random drive for jackal

# imports
import rospy
import random
import sys
import time
import roslaunch
import os

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


# Global variables for random bounds
# overall scale of autodrive. Smaller = slower
scale       =  0.65 

# Initialize globals

# Bounds on angular random value
angular_min =  0
angular_max =  0

# Bounds on linear random value
linear_min  =  0
linear_max  =  0

# Max acceleration figure to smooth motion
linear_acc  =  0.01
angular_acc =  0.005

# Bounds on run counter random value
count_min = 25
count_max = 40

start_time  =  0
escape_command = 0
danger_flag = 0

# Constants for laser averaging
front_delta = 15
side_ang    = 30
side_delta  = 20
side_thresh = 0.875

# Radian to degree function
def toAng(rad):
    ang = rad * 180 / 3.14159
    return ang


# Averaged Sum of scan points function
def getSum(start, end, data):
    angSum = float(0.0)
    index = start
    while index < end :
        if data.ranges[index] < 15:
            angSum = angSum + data.ranges[index]

        index = index + 1

    angSum = float(angSum) / float(end-start)

    return angSum


# Minimum of scan points function
def getMin(start, end, data):
    angSum = float(0.0)
    index = start + 1
    minScan = data.ranges[start]
    while index < end :
        if data.ranges[index] < minScan:
            prev = data.ranges[index]

        index = index + 1

    return minScan


# define callback for twist
def Callback(data):
    global linear_min, linear_max, angular_min, angular_max, escape_command, danger_flag

    # Calculate front, left, and right angles in the data array
    zeroAng    = int((((abs(data.angle_min) + abs(data.angle_max)) / data.angle_increment) / 2) - 1)
    leftAng    = zeroAng + int(side_ang / toAng(data.angle_increment))
    rightAng   = zeroAng - int(side_ang / toAng(data.angle_increment))
    sideOffset = int(side_delta / toAng(data.angle_increment))
    zeroOffset = int(front_delta / toAng(data.angle_increment))
    
    # Compute minimums for left, right, and front laser scan spans
    leftAve  = getMin(leftAng, leftAng + sideOffset, data)
    rightAve = getMin(rightAng - sideOffset, rightAng, data)
    frontAve = getMin(zeroAng - zeroOffset, zeroAng + zeroOffset, data)

    # Output for monitoring
    rospy.loginfo('\t%3.4f  -  %3.4f  -  %3.4f', leftAve, frontAve, rightAve)

    # Set the threshold levels for randomization

    # Too close in front, turn left and slowly back up
    if frontAve < 1 :
        linear_acc  =  1.0 * scale
        angular_acc =  1.0 * scale
        
        angular_min = 0.3 * scale
        angular_max = 0.5  * scale
        
        linear_min  = -0.05 * scale
        linear_max  = 0 * scale
        
        escape_command = 1

    # All Clear, randomly drive forward with varying turn
    elif (frontAve > 1.75) and (leftAve > side_thresh) and (rightAve > side_thresh) :
        linear_acc  =  0.00005 * scale
        angular_acc =  0.00001 * scale
        
        angular_min = -0.625 * scale
        angular_max = 0.625 * scale
        
        linear_min  = 0.75 * scale
        linear_max  = 1.0 * scale
        
        danger_flag = 0

    # Close to a wall on one side, turn to side with most time
    else :
        linear_acc  =  0.00005 * scale
        angular_acc =  0.00001 * scale
        
        escape_command = 1

        if leftAve > rightAve :
            angular_min = 0.75 * scale
            angular_max = 1.0 * scale
            
            linear_min  = 0.25 * scale
            linear_max  = 0.50 * scale
            
        else :
            angular_min = -1.0 * scale
            angular_max = -0.75 * scale
            
            linear_min  = 0.25 * scale
            linear_max  = 0.50 * scale


# define setup and run routine
def setup():
    global start_time, escape_command, danger_flag
    start_time = time.time()

    # create node for listening to twist messages
    rospy.init_node("jackal_map")

    # subscribe to all
    rospy.Subscriber("/scan", LaserScan, Callback)
    # rate = rospy.Rate(user_rate)
    rate = rospy.Rate(100)

    # publish to cmd_vel of the jackal
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Variables for messages and timing
    count = 0
    countLimit = random.randrange(count_min,count_max)
    randLin = float(0.0)
    randAng = float(0.0)
    linSet = float(0.0)
    angSet = float(0.0)

    # loop
    while not time.time()-start_time>(60*7):

        # generate random movement mapping at random interval
        if count < countLimit :
            if (escape_command == 1) and (danger_flag == 0):
                danger_flag = 1
                escape_command = 0
                count = countLimit
            else :
                count = count + 1
        else :
            count = 0
            countLimit = random.randrange(count_min,count_max)
            randLin = random.uniform(linear_min,linear_max)
            randAng = random.uniform(angular_min,angular_max)

        # Basic acceleration code
        if (randLin > linSet):
            if (randLin > (linSet + linear_acc)):
                linSet = linSet + linear_acc
            else :
                linSet = randLin
        else :
            if (randLin < (linSet - linear_acc)):
                linSet = linSet - linear_acc
            else :
                linSet = randLin

        if (randAng > angSet):
            if (randAng > (angSet + angular_acc)):
                angSet = angSet + angular_acc
            else :
                angSet = randAng
        else :
            if (randAng < (angSet - angular_acc)):
                angSet = angSet - angular_acc
            else :
                angSet = randAng

        # push Twist msgs
        linear_msg  = Vector3(x=linSet, y=float(0.0), z=float(0.0))
        angular_msg = Vector3(x=float(0.0), y=float(0.0), z=angSet)
        publish_msg = Twist(linear=linear_msg, angular=angular_msg)

		# publish Twist
        pub.publish(publish_msg)
        pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)

        rate.sleep()


# standard ros boilerplate
if __name__ == "__main__":
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
