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
from app_error_exception import AppError
from helper_functions import CognitiveExerciseHelperFunctions
from rapp_platform_ros_communications.srv import (
  userScoreHistoryForAllCategoriesSrv,
  userScoreHistoryForAllCategoriesSrvResponse
  )
from rapp_platform_ros_communications.msg import (
  CognitiveExercisePerformanceRecordsMsg,
  ArrayCognitiveExercisePerformanceRecordsMsg,
  StringArrayMsg
  )

## @class UserScoreHistoryForAllCategories
# @brief Provides the necessary functions for returning the history of user scores
#
# It implements the cognitive exercise return user score history for all categories service
class UserScoreHistoryForAllCategories:

  ## @brief The callback function of the cognitive exercise user score history for all categories service
  # @param req [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvResponse::Response&] The ROS service response
  # @exception Exception ValueError
  # @exception Exception IndexError
  # @exception Exception AppError
  def returnUserHistory(self,req):
    try:
      res = userScoreHistoryForAllCategoriesSrvResponse() 
      #Validate the provided time range     
      fromTime,toTime=self.validateTimeRange(req.fromTime,req.toTime,res)     
      #Get user ontology alias   
      userOntologyAlias=CognitiveExerciseHelperFunctions.getUserOntologyAlias(req.user_id)    
      #Get test types from ontology     
      validtestTypesList=CognitiveExerciseHelperFunctions.getTestTypesFromOntology()    
      #Construct the test type list for returning their score history 
      testTypesList=CognitiveExerciseHelperFunctions.determineTestTypeListForReturningScoresOrHistory(req.testType,validtestTypesList)      
      res.testCategories=testTypesList      
      #Retrieve the test history for each test category
      self.assignTestHistoryForTestCategoriesToSrv(testTypesList,userOntologyAlias,fromTime,toTime,res)  
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

  ## @brief Retrieves the test history of the user for the provided test categories  
  # @param testTypesList [list] The list of the available tests as they were read from the ontology
  # @param userOntologyAlias [string] The user's ontology alias
  # @param fromTime [long] The time from which the user's performance records are taken into account
  # @param toTime [long] The time up to which the user's performance records are taken into account
  # 
  # @return res [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoreHistoryForAllCategoriesSrv

  def assignTestHistoryForTestCategoriesToSrv(self,testTypesList,userOntologyAlias,fromTime,toTime,res):        
   for s in testTypesList:
      userPerformanceResponse=CognitiveExerciseHelperFunctions.getUserPerformanceRecordsForTestType(s,userOntologyAlias)            
      if(userPerformanceResponse.success==True): 
        userPerfOrganizedByTimestamp=CognitiveExerciseHelperFunctions.organizeUserPerformanceByTimestamp(userPerformanceResponse)
        sumCategoryScore=0
        sumCategoryScoreDivideBy=0;
        countValidTests=0;
        arrayCognitiveExercisePerformanceRecords=ArrayCognitiveExercisePerformanceRecordsMsg()
        for k, v in userPerfOrganizedByTimestamp.items():
          if(k<fromTime or k>toTime):
            del userPerfOrganizedByTimestamp[k]
          else:
            tmpRecord=CognitiveExercisePerformanceRecordsMsg()
            tmpRecord.timestamp=k
            tmpList=v[0][0].split('#') 
            if (tmpList[1] is not None):
              tmpRecord.test=tmpList[1]
            tmpRecord.score=long(v[0][1])           
            tmpRecord.difficulty=v[0][2]
            tmpList=v[0][3].split('#')
            if (tmpList[1] is not None):
              tmpRecord.subtype=tmpList[1]
            tmpRecord.type=s
            countValidTests=countValidTests+1
            sumCategoryScore=sumCategoryScore+long(v[0][2])*long(v[0][1])  #[0][2] is difficulty, [0][1] is score         
            sumCategoryScoreDivideBy=sumCategoryScoreDivideBy+long(v[0][2])            
            tmpAverageCategoryScore=float(sumCategoryScore/sumCategoryScoreDivideBy)
            tmpRecord.meanScoreForTypeUpToNow=tmpAverageCategoryScore
            arrayCognitiveExercisePerformanceRecords.records.append(tmpRecord)
        res.recordsPerTestType.append(arrayCognitiveExercisePerformanceRecords)

  ## @brief Validates the fromTime and toTime variables  
  # @param fromTime [long] The time from which the performance records are evaluated
  # @param toTime [long] The time up to which the performance records are evaluated
  # 
  # @return fromTime [long] The time from which the performance records are evaluated
  # @return toTime [long] The time up to which the performance records are evaluated
  # @exception Exception AppError
  def validateTimeRange(self,fromTime,toTime,res):
    if (toTime is None or toTime==0):
      toTime=9999999999999999999999
    if (fromTime is None or fromTime==0):
      fromTime=0    
    if(fromTime>=toTime):      
      error="Invalid time range, fromTime is later than toTime"
      raise AppError(error,error)     
    return fromTime,toTime
