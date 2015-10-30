#!/usr/bin/env python

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














