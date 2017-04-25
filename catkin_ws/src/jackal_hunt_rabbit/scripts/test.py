#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities


class CommandVelocity():
    """Driving my robot
    """

    def __init__(self):
        rospy.loginfo("Starting node")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) # Creating a publisher with whatever name...
        
    # A function to send velocities until the node is killed
    def send_velocities(self):
        r = rospy.Rate(10) # Setting a rate (hz) at which to publish
        while not rospy.is_shutdown(): # Runnin until killed
            rospy.loginfo("Sending commands")
            twist_msg = Twist() # Creating a new message to send to the robot

            # ... put something relevant into your message
            twist_msg.angular.z = 0.3

            self.pub.publish(twist_msg) # Sending the message via our publisher
            self.pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)
            r.sleep() # Calling sleep to ensure the rate we set above

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
    cv.send_velocities() # Calling the function
    rospy.spin()