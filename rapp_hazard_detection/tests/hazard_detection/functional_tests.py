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
  LightCheckRosSrv,
  LightCheckRosSrvRequest
  )


class LightCheckFunc(unittest.TestCase):
    """Handles the light checking functional tests
    """
    
    ## Tests face detection with lamp turned on
    def test_lightOn(self):
        rospack = rospkg.RosPack()
        light_service = rospy.get_param("rapp_hazard_detection_light_check_topic")
        rospy.wait_for_service(light_service)
        lc_service = rospy.ServiceProxy(light_service, LightCheckRosSrv)
        req = LightCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/hazard_detection_samples/lamp_on.jpg'
        response = lc_service(req)
        light_level = response.light_level
        self.assertGreater( light_level, 50 )
        
    ## Tests face detection with lamp turned off
    def test_lightOff(self):
        rospack = rospkg.RosPack()
        light_service = rospy.get_param("rapp_hazard_detection_light_check_topic")
        rospy.wait_for_service(light_service)
        lc_service = rospy.ServiceProxy(light_service, LightCheckRosSrv)
        req = LightCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/hazard_detection_samples/lamp_off.jpg'
        response = lc_service(req)
        light_level = response.light_level
        self.assertLess( light_level, 50 )

    ## Stress test for face detection. 20 calls in a row
    def test_light_stress(self):
        rospack = rospkg.RosPack()
        light_service = rospy.get_param("rapp_hazard_detection_light_check_topic")
        rospy.wait_for_service(light_service)
        lc_service = rospy.ServiceProxy(light_service, LightCheckRosSrv)
        req = LightCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/hazard_detection_samples/lamp_on.jpg'
        for i in range(0, 20):
            response = lc_service(req)
            light_level = response.light_level
            self.assertGreater( light_level, 50 )

    ## Tests face detection with a non existent image. Should return 0 faces
    def test_fileDoesNotExist(self):
        rospack = rospkg.RosPack()
        light_service = rospy.get_param("rapp_hazard_detection_light_check_topic")
        rospy.wait_for_service(light_service)
        lc_service = rospy.ServiceProxy(light_service, LightCheckRosSrv)
        req = LightCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/not_existent_file.jpg'
        response = lc_service(req)
        light_level = response.light_level
        self.assertEqual( light_level, -1 )

    ## Tests face detection with an audio file. Should not crush an return 0 faces
    def test_fileExistsButItAudio(self):
        rospack = rospkg.RosPack()
        light_service = rospy.get_param("rapp_hazard_detection_light_check_topic")
        rospy.wait_for_service(light_service)
        lc_service = rospy.ServiceProxy(light_service, LightCheckRosSrv)
        req = LightCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/silence_sample.wav'
        response = lc_service(req)
        light_level = response.light_level
        self.assertEqual( light_level, -1 )

## The main function. Initializes the functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'LightCheckFunc', LightCheckFunc)














