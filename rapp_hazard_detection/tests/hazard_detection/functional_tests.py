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
  LightCheckRosSrvRequest,
  DoorCheckRosSrv,
  DoorCheckRosSrvRequest
  )


class HazardDetectionFunc(unittest.TestCase):
    """Handles the hazard detection functional tests
    """
    
    ## Tests light detection with lamp turned on
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
        
    ## Tests light detection with lamp turned off
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

    ## Stress test for light detection. 20 calls in a row
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

    ## Tests light detection with a non existent image. Should return -1
    def test_light_fileDoesNotExist(self):
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

    ## Tests light detection with an audio file. Should not crush an return -1
    def test_light_fileExistsButItAudio(self):
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

    ## Tests door detection with door opened
    def test_doorOpen(self):
        rospack = rospkg.RosPack()
        door_service = rospy.get_param("rapp_hazard_detection_door_check_topic")
        rospy.wait_for_service(door_service)
        lc_service = rospy.ServiceProxy(door_service, DoorCheckRosSrv)
        req = DoorCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/hazard_detection_samples/door_2.png'
        response = lc_service(req)
        door_angle = response.door_angle
        self.assertGreater( door_angle, 1 )
        
    ## Tests door detection with door closed
    def test_doorClosed(self):
        rospack = rospkg.RosPack()
        door_service = rospy.get_param("rapp_hazard_detection_door_check_topic")
        rospy.wait_for_service(door_service)
        lc_service = rospy.ServiceProxy(door_service, DoorCheckRosSrv)
        req = DoorCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/hazard_detection_samples/door_4.png'
        response = lc_service(req)
        door_angle = response.door_angle
        self.assertLess( door_angle, 2 )

    ## Stress test for door detection. 20 calls in a row
    def test_door_stress(self):
        rospack = rospkg.RosPack()
        door_service = rospy.get_param("rapp_hazard_detection_door_check_topic")
        rospy.wait_for_service(door_service)
        lc_service = rospy.ServiceProxy(door_service, DoorCheckRosSrv)
        req = DoorCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/hazard_detection_samples/door_1.png'
        for i in range(0, 20):
            response = lc_service(req)
            door_angle = response.door_angle
            self.assertGreater( door_angle, 0 )

    ## Tests door detection with a non existent image. Should return -1
    def test_door_fileDoesNotExist(self):
        rospack = rospkg.RosPack()
        door_service = rospy.get_param("rapp_hazard_detection_door_check_topic")
        rospy.wait_for_service(door_service)
        lc_service = rospy.ServiceProxy(door_service, DoorCheckRosSrv)
        req = DoorCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/not_existent_file.jpg'
        response = lc_service(req)
        door_angle = response.door_angle
        self.assertEqual( door_angle, -1 )

    ## Tests door detection with an audio file. Should not crush an return -1
    def test_door_fileExistsButItAudio(self):
        rospack = rospkg.RosPack()
        door_service = rospy.get_param("rapp_hazard_detection_door_check_topic")
        rospy.wait_for_service(door_service)
        lc_service = rospy.ServiceProxy(door_service, DoorCheckRosSrv)
        req = DoorCheckRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/silence_sample.wav'
        response = lc_service(req)
        door_angle = response.door_angle
        self.assertEqual( door_angle, -1 )


## The main function. Initializes the functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'HazardDetectionFunc', HazardDetectionFunc)














