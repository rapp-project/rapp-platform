#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr

import rospy
from collections import OrderedDict
from helper_functions import CognitiveExerciseHelperFunctions
from app_error_exception import AppError

from rapp_platform_ros_communications.srv import (
  userScoresForAllCategoriesSrvResponse
  )
## @class UserScoresForAllCategories
# @brief Provides the necessary functions for returning the user scores
#
# It implements the user scores for all categories service
class UserScoresForAllCategories:

  ## @brief The callback function of the cognitive exercise user scores for all categories service
  # @param req [rapp_platform_ros_communications::userScoresForAllCategoriesSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::userScoresForAllCategoriesSrvResponse::Response&] The ROS service response
  # @exception Exception ValueError
  # @exception Exception IndexError
  # @exception Exception AppError
  def returnUserScores(self,req):
    try:
      res = userScoresForAllCategoriesSrvResponse()      
      #Get user ontology alias
      userOntologyAlias=CognitiveExerciseHelperFunctions.getUserOntologyAlias(req.username)
      #Get test types from ontology       
      validtestTypesList=CognitiveExerciseHelperFunctions.getTestTypesFromOntology()
      #Construct the test type list for returning their score history    
      testTypesList=CognitiveExerciseHelperFunctions.determineTestTypeListForReturningScoresOrHistory(req.testType,validtestTypesList)  
      res.testCategories=testTypesList
      #Calculate scores for test categories
      res.testScores=self.calculateUserScoresForCategories(testTypesList,userOntologyAlias,req.upToTime)   
      res.success=True
    except ValueError:
      res.trace.append("ValueError: " +str(e))
      res.error="ValueError: "+str(e)
      res.success=False
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.error="IndexError: "+str(e)
      res.success=False
    except AppError as e:
      AppError.passErrorToRosSrv(e,res) 
    return res  

  ## @brief Calculates and returns the user's scores for the provided test types  
  # @param testTypesList [list] The list of the available tests as they were read from the ontology
  # @param userOntologyAlias [string] The user's ontology alias
  # @param upToTime [long] The time up to which the user's performance records are taken into account
  # 
  # @return testScores [list] The test scores for each category
  def calculateUserScoresForCategories(self,testTypesList,userOntologyAlias,upToTime):    
    scoresPerCategory=[]
    d1=OrderedDict()
    for s in testTypesList:
      userPerformanceResponse=CognitiveExerciseHelperFunctions.getUserPerformanceRecordsForTestType(s,userOntologyAlias)      
      if(userPerformanceResponse.success!=True):
        scoresPerCategory.append(0)
      else:
        scoreOfCategory=0
        scoreOfCategoryDivideBy=0
        for i in range(len(userPerformanceResponse.tests)):
          if(int(userPerformanceResponse.timestamps[i])<upToTime):
            scoreOfCategory=scoreOfCategory+(int(userPerformanceResponse.scores[i])*int(userPerformanceResponse.difficulty[i]))            
            scoreOfCategoryDivideBy=scoreOfCategoryDivideBy+int(userPerformanceResponse.difficulty[i])
        #scoresPerCategory.append(long(scoreOfCategory/len(userPerformanceResponse.tests)))
        if(scoreOfCategoryDivideBy==0):
          scoresPerCategory.append(0)
        else:
          scoresPerCategory.append(float(scoreOfCategory/scoreOfCategoryDivideBy))
    return scoresPerCategory    
