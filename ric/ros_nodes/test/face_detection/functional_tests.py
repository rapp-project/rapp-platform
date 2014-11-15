#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest
import rospy
from rapp_platform_ros_communications.srv import *

class FaceDetFunc(unittest.TestCase):

    def test_faceDetectionFunctional(self):
        rospy.wait_for_service('ric/face_detection_service')
        fd_service = rospy.ServiceProxy('ric/face_detection_service', FaceDetectionRosSrv)
        req = FaceDetectionRosSrv.Request
        req.imageFilename = "/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png"
        response = fd_service(req)
        faces_num = len(response.faces_up_left)
        self.assertEqual( faces_num, 1, "Face detection ok")
       
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'FaceDetFunc', FaceDetFunc)














