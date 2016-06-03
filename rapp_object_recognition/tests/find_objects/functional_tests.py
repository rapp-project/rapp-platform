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
  FindObjectsSrv,
  FindObjectsSrvRequest,
  ClearModelsSrv,
  ClearModelsSrvRequest,
  LearnObjectSrv,
  LearnObjectSrvRequest,
  LoadModelsSrv,
  LoadModelsSrvRequest
  )


class ObjectRecognitionFunc(unittest.TestCase):
    """Handles the object recognition functional tests
    """
    
    ## Tests object model learning and loading.
    def test_learn(self):
        rospack = rospkg.RosPack()
        
        clear_service_name = rospy.get_param("rapp_object_clear_topic")
        learn_service_name = rospy.get_param("rapp_object_learn_topic")
        load_service_name = rospy.get_param("rapp_object_load_topic")
        find_service_name = rospy.get_param("rapp_object_recognition_topic")
        
        rospy.wait_for_service(clear_service_name)
        rospy.wait_for_service(learn_service_name)
        rospy.wait_for_service(load_service_name)
        rospy.wait_for_service(find_service_name)
        
        clear_service = rospy.ServiceProxy(clear_service_name, ClearModelsSrv)
        learn_service = rospy.ServiceProxy(learn_service_name, LearnObjectSrv)
        load_service = rospy.ServiceProxy(load_service_name, LoadModelsSrv)
        find_service = rospy.ServiceProxy(find_service_name, FindObjectsSrv)
        
        req = ClearModelsSrvRequest()
        req.user = 'test'
        response = clear_service(req)
        self.assertEqual(response.result, 0)
        
        req = LearnObjectSrvRequest()
        req.user = 'test'
        req.name = 'cat'
        req.fname = rospack.get_path('rapp_testing_tools') + \
                '/test_data/object_recognition_samples/book_1/cat.jpg'
        response = learn_service(req)
        self.assertEqual(response.result, 0)
        
        req.name = 'catfoo'
        req.fname = rospack.get_path('rapp_testing_tools') + \
                '/test_data/object_recognition_samples/book_1/catfoo.jpg'
        response = learn_service(req)
        self.assertEqual(response.result, -2)
        
        req = FindObjectsSrvRequest()
        req.user = 'test'
        req.fname = rospack.get_path('rapp_testing_tools') + \
                '/test_data/object_recognition_samples/book_1/cat.jpg'
        req.limit = 1
        response = find_service(req)
        self.assertEqual(response.result, -1)
        
        req = LoadModelsSrvRequest()
        req.user = 'test'
        req.names = ['cat', 'catfoo']
        response = load_service(req)
        self.assertEqual(response.result, 0)
        

## The main function. Initializes the functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'ObjectRecognitionFunc', ObjectRecognitionFunc)














