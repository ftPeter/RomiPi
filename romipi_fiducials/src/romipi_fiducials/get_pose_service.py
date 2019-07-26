#!/usr/bin/env python
# get_pose_service.py

from romipi_fiducials.srv import GetPose, GetPoseResponse
from geometry_msgs.msg import PoseStamped
import rospy

from romipi_fiducials.store_poses import PoseStorage

class GetPoseService():
    def handle_get_pose(self, req):
        """ look for pose and return found True, otherwise return found False """
        name = req.name
        found = False
        # search for pose
        pose = self.storage.retrieve(name)
        if pose is not None:
            found = True 
            return GetPoseResponse(found, pose)
        else:
            # pose not found!
            return GetPoseResponse(found, PoseStamped())

    def store_pose(self, name, pose):
        self.storage.store(name, pose)
        return

    def start_get_pose_service(self):
        self.storage = PoseStorage()
        """ start the get pose service """
        s = rospy.Service('~get_pose', GetPose, self.handle_get_pose)

def _get_pose_server():
    rospy.init_node('get_pose_server')
    start_get_pose_service()
    rospy.spin()

if __name__ == "__main__":
    _get_pose_server()
