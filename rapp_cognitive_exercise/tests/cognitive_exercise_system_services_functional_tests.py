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
  testSelectorSrvResponse,
  cognitiveTestCreatorSrv,
  cognitiveTestCreatorSrvRequest,
  cognitiveTestCreatorSrvResponse,
  userScoresForAllCategoriesSrv,
  userScoresForAllCategoriesSrvRequest,
  userScoresForAllCategoriesSrvResponse,
  userScoreHistoryForAllCategoriesSrv,
  userScoreHistoryForAllCategoriesSrvRequest,
  userScoreHistoryForAllCategoriesSrvResponse
  )

## @class CognitiveExerciseFunc 
# Inherits the unittest.TestCase class in order to offer functional tests functionality 
class CognitiveExerciseFunc(unittest.TestCase):

    ## Tests the cognitive exercise test chooser service when test type is provided
    def test_chooser_basic(self):
        ros_service = rospy.get_param(\
                "rapp_cognitive_exercise_chooser_topic")
        rospy.wait_for_service(ros_service)
        
        test_service = rospy.ServiceProxy(\
                ros_service, testSelectorSrv)

        req = testSelectorSrvRequest()
        req.username="rapp"        
        req.testType="ArithmeticCts"
        response = test_service(req)     
        self.assertEqual(response.success, True)  
        
    ## Tests the cognitive exercise test chooser service when test type is not provided  
    def test_chooser_no_type(self):
        ros_service = rospy.get_param(\
                "rapp_cognitive_exercise_chooser_topic")
        rospy.wait_for_service(ros_service)
        
        test_service = rospy.ServiceProxy(\
                ros_service, testSelectorSrv)
    
        req = testSelectorSrvRequest()
        req.username="rapp"        
        req.testType=""
        response = test_service(req)     
        self.assertEqual(response.success, True)    
              
    ## Tests the record user cognitive exercise test performance service  
    def test_record_user_performance_basic(self):
        ros_service = rospy.get_param(\
                "rapp_cognitive_exercise_record_user_cognitive_test_performance_topic")
        rospy.wait_for_service(ros_service)
        
        test_service = rospy.ServiceProxy(\
                ros_service, recordUserCognitiveTestPerformanceSrv)

        req = recordUserCognitiveTestPerformanceSrvRequest()
        req.username="rapp"        
        req.test="ArithmeticCts_stXqnGrc"
        req.score=10
        response = test_service(req)     
        self.assertEqual(response.success, True)  
        
    def test_cognitive_test_creat_of_invalid_path(self):
        ros_service = rospy.get_param(\
                "rapp_cognitive_test_creator_topic")
        rospy.wait_for_service(ros_service)
        
        test_service = rospy.ServiceProxy(\
                ros_service, cognitiveTestCreatorSrv)

        req = cognitiveTestCreatorSrvRequest()
        req.inputFile="someInvalidPath"
        response = test_service(req)     
        self.assertEqual(response.success, False)  
        self.assertEqual(response.error, "IO Error, cannot open test file or write xml file")
        
    def test_user_scores_for_all_categories_valid_input(self):
        ros_service = rospy.get_param(\
                "rapp_cognitive_exercise_user_all_categories_score_topic")
        rospy.wait_for_service(ros_service)
        
        test_service = rospy.ServiceProxy(\
                ros_service, userScoresForAllCategoriesSrv)

        req = userScoresForAllCategoriesSrvRequest()
        req.username="rapp"
        req.upToTime=10000000000000
        response = test_service(req)     
        self.assertEqual(response.success, True)          
        self.assertEqual("ArithmeticCts" and "ReasoningCts" and "AwarenessCts" in response.testCategories, True)
        self.assertEqual(len(response.testScores)>=3, True)

    def test_user_score_history_for_all_categories_valid_input_basic(self):
        ros_service = rospy.get_param(\
                "rapp_cognitive_exercise_user_all_categories_history_topic")
        rospy.wait_for_service(ros_service)
        
        test_service = rospy.ServiceProxy(\
                ros_service, userScoreHistoryForAllCategoriesSrv)

        req = userScoreHistoryForAllCategoriesSrvRequest()
        req.username="rapp"
        req.toTime=0
        req.fromTime=0
        response = test_service(req)     
        self.assertEqual(response.success, True)          
        self.assertEqual(len(response.recordsPerTestType)>1,True)
        
## The main function. Initializes the Cognitive Exercise System functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'CognitiveExerciseFunc', CognitiveExerciseFunc)














