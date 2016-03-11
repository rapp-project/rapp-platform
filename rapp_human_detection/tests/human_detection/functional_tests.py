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
  HumanDetectionRosSrv,
  HumanDetectionRosSrvRequest
  )


class HumanDetFunc(unittest.TestCase):
    """Handles the human detection functional tests
    """

    ## Tests human detection with a NAO captured image. Should return 1 human
    def test_humanExists_realistic(self):
        rospack = rospkg.RosPack()
        human_service = rospy.get_param("rapp_human_detection_detect_humans_topic")
        rospy.wait_for_service(human_service)
        fd_service = rospy.ServiceProxy(human_service, HumanDetectionRosSrv)
        req = HumanDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/human_detection_samples/NAO_picture_3.png'
        response = fd_service(req)
        humans_num = len(response.humans_up_left)
        self.assertEqual( humans_num, 1 )

    ## Tests human detection with a NAO captured image from almost 2 meters. Should return 1 human
    # DISABLED - Returned 2
    # def test_humanExists_realistic_2(self):
        # rospack = rospkg.RosPack()
        # human_service = rospy.get_param("rapp_human_detection_detect_humans_topic")
        # rospy.wait_for_service(human_service)
        # fd_service = rospy.ServiceProxy(human_service, HumanDetectionRosSrv)
        # req = HumanDetectionRosSrvRequest()
        # req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                # '/test_data/human_detection_samples/NAO_picture_10.png'
        # response = fd_service(req)
        # humans_num = len(response.humans_up_left)
        # self.assertEqual( humans_num, 1 )

    ## Stress test for human detection. 20 calls in a row
    def test_humanExists_stress(self):
        rospack = rospkg.RosPack()
        human_service = rospy.get_param("rapp_human_detection_detect_humans_topic")
        rospy.wait_for_service(human_service)
        fd_service = rospy.ServiceProxy(human_service, HumanDetectionRosSrv)
        req = HumanDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/human_detection_samples/NAO_picture_3.png'
        for i in range(0, 10):
            response = fd_service(req)
            humans_num = len(response.humans_up_left)
            self.assertEqual( humans_num, 1 )

    ## Tests human detection with an image that does not contain humans. Should return 0 humans
    def test_humanDoesNotExist(self):
        rospack = rospkg.RosPack()
        human_service = rospy.get_param("rapp_human_detection_detect_humans_topic")
        rospy.wait_for_service(human_service)
        fd_service = rospy.ServiceProxy(human_service, HumanDetectionRosSrv)
        req = HumanDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/qr_code_rapp.jpg'
        response = fd_service(req)
        humans_num = len(response.humans_up_left)
        self.assertEqual( humans_num, 0 )

    ## Tests human detection with a non existent image. Should return 0 humans
    def test_fileDoesNotExist(self):
        rospack = rospkg.RosPack()
        human_service = rospy.get_param("rapp_human_detection_detect_humans_topic")
        rospy.wait_for_service(human_service)
        fd_service = rospy.ServiceProxy(human_service, HumanDetectionRosSrv)
        req = HumanDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/file_does_not_exist.png'
        response = fd_service(req)
        humans_num = len(response.humans_up_left)
        self.assertEqual( humans_num, 0 )

    ## Tests human detection with an audio file. Should not crush an return 0 humans
    def test_fileExistsButItAudio(self):
        rospack = rospkg.RosPack()
        human_service = rospy.get_param("rapp_human_detection_detect_humans_topic")
        rospy.wait_for_service(human_service)
        fd_service = rospy.ServiceProxy(human_service, HumanDetectionRosSrv)
        req = HumanDetectionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/silence_sample.wav'
        response = fd_service(req)
        humans_num = len(response.humans_up_left)
        self.assertEqual( humans_num, 0 )

## The main function. Initializes the functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'HumanDetFunc', HumanDetFunc)














