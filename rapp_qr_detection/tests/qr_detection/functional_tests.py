#!/usr/bin/env python

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

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

## @class QrDetFunc
# Supports the deployment of QR detection functional tests
class QrDetFunc(unittest.TestCase):
    
    ## Tests QR detection with an image containing a QR. Should return 1 qr
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

    ## Stress test for the QR detection. Performs 100 consecutive calls
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

    ## Tests qr detection with an image containing a face. Should return 0 qrs
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

    ## Tests qr detection with a non-existent image. Should return 0 qrs
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

    ## Tests qr detection with an audio file. Should return 0 qrs
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

## The main function. Initializes the functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'QrDetFunc', QrDetFunc)

