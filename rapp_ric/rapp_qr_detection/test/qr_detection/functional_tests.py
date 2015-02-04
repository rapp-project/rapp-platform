#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest
import rospy
from rapp_platform_ros_communications.srv import (
  QrDetectionRosSrv,
  QrDetectionRosSrvRequest
  )


class QrDetFunc(unittest.TestCase):

    def test_qrDetectionFunctional(self):
        rospy.wait_for_service('ric/ros_nodes/qr_detection_service')
        fd_service = rospy.ServiceProxy('ric/ros_nodes/qr_detection_service', \
            QrDetectionRosSrv)
        req =QrDetectionRosSrvRequest()
        req.imageFilename = \
            "/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform/ric/test_auxiliary_files/qr_code_rapp.jpg"
        response = fd_service(req)
        num = len(response.qr_messages)
        self.assertEqual( num, 1, "Qr detection ok")
       
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'QrDetFunc', QrDetFunc)

