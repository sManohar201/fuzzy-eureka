#!/usr/bin/env python

# ----------------------------------------------------------
# FINAL PROJECT - EE5900 Intro to Robotics
# State machine implementation for the project
# Team Members
#   1) Deep Doshi (Team Lead)
#   2) Haden Wasserbaech
#   3) Sabari Manohar
# ----------------------------------------------------------

# load the dependencies for the project
from smach import StateMachine, State, Concurrence
import smach_ros  #### donot know why it is loaded
import rospy
# This is the message type the robot uses for velocities
from geometry_msgs.msg import Twist, Vector3
# This is the message type the robot uses for laserscan
from sensor_msgs.msg import LaserScan
# This is the message type the robot uses for odometry measure
from nav_msgs.msg import Odometry
# numpy library for array manipulation
import numpy as np
import roslib
import math
import random
import sys
import time
import roslaunch
import os

global map_loop
map_loop = 0

# Global variables for random bounds
scale = 0.7
angular_min = -0.2
linear_min = -0.3
angular_max = 0.2
linear_max = 0.3


# Constants for laser averaging
front_delta = 15
side_ang = 30
side_delta = 15
side_thresh = 1.25

# increment value at each time step
rate_step = 0.15


# linear accleration and decceleration
def smooth_vel(vel_before, vel_final, t_before, t_final, rate):
    step = rate * (t_final - t_before)
    sign = 1.0 if (vel_final > vel_before) else -1.0
    error = math.fabs(vel_final - vel_before)
    if error < step:
        return vel_final
    else:
        return vel_before + sign * step


# Radian to degree function
def toAng(rad):
    ang = rad * 180 / 3.14
    return ang


# Averaged Sum of scan points function
def getSum(start, end, data):
    angSum = float(0.0)
    index = start
    while index < end:
        if data.ranges[index] < 15:
            angSum = angSum + data.ranges[index]

        index = index + 1

    angSum = float(angSum) / float(end - start)

    return angSum


# Averaged Sum of scan points function
def getMin(start, end, data):
    angSum = float(0.0)
    index = start + 1
    minScan = data.ranges[start]
    while index < end:
        if data.ranges[index] < minScan:
            prev = data.ranges[index]

        index = index + 1

    return minScan


""" EXPLORER -- LASERSCAN"""


# Subscriber callback function
# define callback for twist
def Callback(data):
    global linear_min, linear_max, angular_min, angular_max

    # Calculate front, left, and right angles in the data array
    zeroAng = int((((abs(data.angle_min) + abs(data.angle_max)) / data.angle_increment) / 2) - 1)
    leftAng = zeroAng + int(side_ang / toAng(data.angle_increment))
    rightAng = zeroAng - int(side_ang / toAng(data.angle_increment))
    sideOffset = int(side_delta / toAng(data.angle_increment))
    zeroOffset = int(front_delta / toAng(data.angle_increment))

    # Compute averages for left, right, and front laser scan spans
    leftAve = getMin(leftAng, leftAng + sideOffset, data)
    rightAve = getMin(rightAng - sideOffset, rightAng, data)
    frontAve = getMin(zeroAng - zeroOffset, zeroAng + zeroOffset, data)

    # Too close in front, turn left and slowly back up
    if frontAve < 1:
        angular_min = 0.25 * scale
        angular_max = 0.5 * scale
        linear_min = -0.05 * scale
        linear_max = 0 * scale

    # All Clear, randomly drive forward with varying turn
    elif (frontAve > 3) and (leftAve > side_thresh) and (rightAve > side_thresh):
        angular_min = -0.5 * scale
        angular_max = 0.5 * scale
        linear_min = 0.50 * scale
        linear_max = 0.7 * scale

    # Close to a wall on one side, turn to side with most time
    else:
        if leftAve > rightAve:
            angular_min = 0.5 * scale
            angular_max = 0.75 * scale
            linear_min = 0.25 * scale
            linear_max = 0.5 * scale
        else:
            angular_min = -1.0 * scale
            angular_max = -0.5 * scale
            linear_min = 0.25 * scale
            linear_max = 0.50 * scale


class Explorer(State):
    """Exploration of the area."""

    def __init__(self):
        State.__init__(self, outcomes=['ExploredArea', 'Break'])

    def execute(self, userdata):
        global ramp_timeL, rate_step

        # subscribe to all
        rospy.Subscriber("/scan", LaserScan, Callback)
        # rate = rospy.Rate(user_rate)
        rate = rospy.Rate(50)

        # publish to cmd_vel of the jackal
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Variables for messages and timing
        count = 0
        countLimit = random.randrange(5, 35)
        randLin = float(0.0)
        randAng = float(0.0)

        # initial values for twist msgs to zero
        twist_init = Twist()
        randLin_n = twist_init.linear.x
        randAng_n = twist_init.angular.z

        #### TRY TO GET THE OUTCOME OF THE ALVAR DETECTOR
        for i in range(450):
            # get a new time stamp
            ramp_timeF = time.time()
            # generate random movement mapping at random interval
            if count < countLimit:
                count = count + 1
            else:
                count = 0
                countLimit = random.randrange(5, 25)
                randLin = random.uniform(linear_min, linear_max)
                randAng = random.uniform(angular_min, angular_max)

            # pass linear and angular velocity values to the smooth_vel function
            # untill randLin and randLin_n matches
            if not (randLin == randLin_n):
                randLin_n = smooth_vel(randLin_n, randLin, ramp_timeL, ramp_timeF, rate_step)
            else:
                randLin_n = randLin

            # push Twist msgs

            linear_msg = Vector3(x=randLin_n*0.65, y=float(0.0), z=float(0.0))

            angular_msg = Vector3(x=float(0.0), y=float(0.0), z=randAng*0.65)
            publish_msg = Twist(linear=linear_msg, angular=angular_msg)

            ramp_timeL = ramp_timeF
            # publish Twist
            pub.publish(publish_msg)
            pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size = 10)

            rate.sleep()

        map_loop += 1

        if map_loop == 2:
            return 'ExploredArea'
        else:
            return 'Break'


class Alvar(State):
    """Alvar detection while exploration."""

    def __init__(self):
        State.__init__(self, outcomes=['AlvarTracked'])

    def execute(self, userdata):
        rospy.loginfo()


class Rotate_anti(State):
    def __init__(self):
        State.__init__(self, outcomes=['RotateAnti'])

    def execute(self, userdata):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rate = rospy.Rate(10)
        for i in range(120):
            twist_msg = Twist()
            twist_msg.angular.z = -0.5
            pub.publish(twist_msg)
            pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=5)
            rate.sleep()

        return 'RotateAnti'


def main():
    rospy.init_node('StateMachine')

    sm = StateMachine(outcomes=['FinalSuccess'])

    with sm:
        StateMachine.add('Clock', Explorer(), transitions={'Break':'AntiClock', 'ExploredArea': 'FinalSuccess'})
        StateMachine.add('AntiClock', Rotate_anti(), transitions={'RotateAnti': 'Clock'})

    sm.execute()


if __name__ == '__main__':
    main()
