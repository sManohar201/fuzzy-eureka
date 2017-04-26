#!/usr/bin/env python

import rospy
import roslaunch
import os

rospy.init_node('en_Mapping', anonymous=True)
# rospy.on_shutdown(self.shutdown)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/robotics4/fuzzy-eureka/catkin_ws/src/jackal_hunt_rabbit/scripts/sample_smach.py"])

launch.start()

# launch.shutdown()
