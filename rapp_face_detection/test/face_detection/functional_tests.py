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
  FaceDetectionRosSrv,
  FaceDetectionRosSrvRequest
  )


class FaceDetFunc(unittest.TestCase):

    def test_faceExists(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_detection_detect_faces_topic")
        rospy.wait_for_service(face_service)
        fd_service = rospy.ServiceProxy(face_service, FaceDetectionRosSrv)
        req = FaceDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/Lenna.png'
        response = fd_service(req)
        faces_num = len(response.faces_up_left)
        self.assertEqual( faces_num, 1 )

    def test_faceExists_realistic(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_detection_detect_faces_topic")
        rospy.wait_for_service(face_service)
        fd_service = rospy.ServiceProxy(face_service, FaceDetectionRosSrv)
        req = FaceDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/face_samples/etsardou_medium.jpg'
        response = fd_service(req)
        faces_num = len(response.faces_up_left)
        self.assertEqual( faces_num, 1 )

    def test_faceExists_realistic_2(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_detection_detect_faces_topic")
        rospy.wait_for_service(face_service)
        fd_service = rospy.ServiceProxy(face_service, FaceDetectionRosSrv)
        req = FaceDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/face_samples/klpanagi_medium_straight.jpg'
        response = fd_service(req)
        faces_num = len(response.faces_up_left)
        self.assertEqual( faces_num, 1 )

    def test_faceExists_stress(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_detection_detect_faces_topic")
        rospy.wait_for_service(face_service)
        fd_service = rospy.ServiceProxy(face_service, FaceDetectionRosSrv)
        req = FaceDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/Lenna.png'
        for i in range(0, 20):
            response = fd_service(req)
            faces_num = len(response.faces_up_left)
            self.assertEqual( faces_num, 1 )

    def test_faceDoesNotExist(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_detection_detect_faces_topic")
        rospy.wait_for_service(face_service)
        fd_service = rospy.ServiceProxy(face_service, FaceDetectionRosSrv)
        req = FaceDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/qr_code_rapp.jpg'
        response = fd_service(req)
        faces_num = len(response.faces_up_left)
        self.assertEqual( faces_num, 0 )

    def test_fileDoesNotExist(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_detection_detect_faces_topic")
        rospy.wait_for_service(face_service)
        fd_service = rospy.ServiceProxy(face_service, FaceDetectionRosSrv)
        req = FaceDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/qr_code_rapp.png'
        response = fd_service(req)
        faces_num = len(response.faces_up_left)
        self.assertEqual( faces_num, 0 )

    def test_fileExistsButItAudio(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_detection_detect_faces_topic")
        rospy.wait_for_service(face_service)
        fd_service = rospy.ServiceProxy(face_service, FaceDetectionRosSrv)
        req = FaceDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/silence_sample.wav'
        response = fd_service(req)
        faces_num = len(response.faces_up_left)
        self.assertEqual( faces_num, 0 )


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'FaceDetFunc', FaceDetFunc)














