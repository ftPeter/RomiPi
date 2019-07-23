#!/usr/bin/env python
#
# store_poses.py
#
# stores most recent pose observation
# 
# Peter F. Klemperer
# June 22, 2019
#
# 

import rospy

# message types
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from romipi_msgs.msg import RomiPose

class PoseStorage():
    def __init__(self):
        self.poses = {}

    def store(self, name, pose):
        self.poses[name] = pose

    def retrieve(self, name):
        if name in self.poses.keys():
            return self.poses[name]
        else:
            return None


