#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest
import rospy
import roslib
import rospkg

from rapp_platform_ros_communications.srv import (
  QrDetectionRosSrv,
  QrDetectionRosSrvRequest
  )

class QrDetFunc(unittest.TestCase):

    def test_qrExists(self):
        rospack = rospkg.RosPack()
        qr_service = rospy.get_param("rapp_qr_detection_detect_qrs_topic")
        rospy.wait_for_service(qr_service)
        fd_service = rospy.ServiceProxy(qr_service, QrDetectionRosSrv)
        req = QrDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/qr_code_rapp.jpg'
        response = fd_service(req)
        qr_num = len(response.qr_messages)
        self.assertEqual( qr_num, 1 )
 
    def test_qrExists_stress(self):
        rospack = rospkg.RosPack()
        qr_service = rospy.get_param("rapp_qr_detection_detect_qrs_topic")
        rospy.wait_for_service(qr_service)
        fd_service = rospy.ServiceProxy(qr_service, QrDetectionRosSrv)
        req = QrDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/qr_code_rapp.jpg'
        for i in range(0,100):
            response = fd_service(req)
            qr_num = len(response.qr_messages)
            self.assertEqual( qr_num, 1 )
 
    def test_qrNotExists(self):
        rospack = rospkg.RosPack()
        qr_service = rospy.get_param("rapp_qr_detection_detect_qrs_topic")
        rospy.wait_for_service(qr_service)
        fd_service = rospy.ServiceProxy(qr_service, QrDetectionRosSrv)
        req = QrDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/Lenna.png'
        response = fd_service(req)
        qr_num = len(response.qr_messages)
        self.assertEqual( qr_num, 0 )

    def test_fileNotExists(self):
        rospack = rospkg.RosPack()
        qr_service = rospy.get_param("rapp_qr_detection_detect_qrs_topic")
        rospy.wait_for_service(qr_service)
        fd_service = rospy.ServiceProxy(qr_service, QrDetectionRosSrv)
        req = QrDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/testingfile.png'
        response = fd_service(req)
        qr_num = len(response.qr_messages)
        self.assertEqual( qr_num, 0 )

    def test_fileIsAudio(self):
        rospack = rospkg.RosPack()
        qr_service = rospy.get_param("rapp_qr_detection_detect_qrs_topic")
        rospy.wait_for_service(qr_service)
        fd_service = rospy.ServiceProxy(qr_service, QrDetectionRosSrv)
        req = QrDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/nai_sample.wav'
        response = fd_service(req)
        qr_num = len(response.qr_messages)
        self.assertEqual( qr_num, 0 )

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'QrDetFunc', QrDetFunc)

