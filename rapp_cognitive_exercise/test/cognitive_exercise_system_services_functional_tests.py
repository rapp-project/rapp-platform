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


PKG='rapp_cognitive_exercise'

import sys
import unittest
import rospy
import roslib

from rapp_platform_ros_communications.srv import (
  recordUserCognitiveTestPerformanceSrv,
  recordUserCognitiveTestPerformanceSrvRequest,
  recordUserCognitiveTestPerformanceSrvResponse,
  testSelectorSrv,
  testSelectorSrvRequest,
  testSelectorSrvResponse
  )

class CognitiveExerciseFunc(unittest.TestCase):

    # Subclasses_of tests
    def test_chooser_basic(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_cognitive_exercise_chooser_topic")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, testSelectorSrv)

        req = testSelectorSrvRequest()
        req.username="rapp"        
        req.testType="ArithmeticCts"
        response = test_service(req)     
        self.assertEqual(response.success, True)  
        
    def test_chooser_no_type(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_cognitive_exercise_chooser_topic")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, testSelectorSrv)
    
        req = testSelectorSrvRequest()
        req.username="rapp"        
        req.testType=""
        response = test_service(req)     
        self.assertEqual(response.success, True)  
        
        
    def test_record_user_performance_basic(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_cognitive_exercise_record_user_cognitive_test_performance_topic")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, recordUserCognitiveTestPerformanceSrv)

        req = recordUserCognitiveTestPerformanceSrvRequest()
        req.username="rapp"        
        req.test="ArithmeticCts_stXqnGrc"
        req.score=10
        response = test_service(req)     
        self.assertEqual(response.success, True)  
  
    
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'CognitiveExerciseFunc', CognitiveExerciseFunc)














