#!/usr/bin/env python

import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
rospy.on_shutdown(self.shutdown)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["$(find jackal_hunt_rabbit)/launch/map.launch"])

launch.start()

launch.shutdown()
