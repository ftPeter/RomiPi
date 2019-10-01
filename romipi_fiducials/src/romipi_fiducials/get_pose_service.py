#!/usr/bin/env python
# get_pose_service.py

import rospy

from romipi_fiducials.srv import GetPose, GetPoseResponse
from romipi_fiducials.srv import ResetVisible, ResetVisibleResponse
from romipi_fiducials.srv import GetVisible, GetVisibleResponse

from romipi_msgs.msg import NameList
from geometry_msgs.msg import PoseStamped

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

    def handle_reset_visible(self, req):
        self.storage.reset()
        return ResetVisibleResponse()

    def handle_get_visible(self, req):
        resp = NameList()
        resp.names = self.storage.get_visible()
        return GetVisibleResponse( resp )

    def store_pose(self, name, pose):
        self.storage.store(name, pose)
        return

    def start_get_pose_service(self):
        self.storage = PoseStorage()
        """ start the get pose service """
        s = rospy.Service('~reset', ResetVisible, self.handle_reset_visible)
        s = rospy.Service('~get_visible', GetVisible, self.handle_get_visible)
        s = rospy.Service('~get_pose', GetPose, self.handle_get_pose)

    def _test_pose_server(self):
        rospy.init_node('get_pose_server')
        self.start_get_pose_service()
        rospy.spin()

if __name__ == "__main__":
    pose_server = GetPoseService()
    pose_server._test_pose_server()

