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
import sys
import calendar
import time
from datetime import datetime
from os.path import expanduser
from collections import OrderedDict



from rapp_platform_ros_communications.srv import (
  ontologySubSuperClassesOfSrv,
  ontologySubSuperClassesOfSrvRequest,
  ontologySubSuperClassesOfSrvResponse,
  createOntologyAliasSrv,
  createOntologyAliasSrvRequest,
  createOntologyAliasSrvResponse,
  userScoreHistoryForAllCategoriesSrv,
  userScoreHistoryForAllCategoriesSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

## @class UserScoreHistoryForAllCategories
# @brief Provides the necessary functions for returning the history of user scores
#
# It implements the cognitive exercise return user score history for all categories service
class UserScoreHistoryForAllCategories:

  ## @brief The callback function of the cognitive exercise user score history for all categories service
  # @param req [rapp_platform_ros_communications::userScoresForAllCategoriesSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::userScoresForAllCategoriesSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception AIOError
  # @exception Exception ValueError
  def returnUserHistory(self,req):
    try:
      res = userScoreHistoryForAllCategoriesSrvResponse()      
      print "akraio"
      
      returnWithError,userOntologyAlias=self.getUserOntologyAlias(req.username,res)
      if(returnWithError):
        return res      
        
      #returnWithError,testTypesList=self.getTestTypesFromOntology(res)
      #if(returnWithError):
        #return res
        
      #res.testCategories=testTypesList
      #res.testScores=self.calculateUserScoresForCategories(testTypesList,userOntologyAlias,req.upToTime)   
      #res.success=True         

    except ValueError:
      res.trace.append("Value Error, probably conversion from integer to string failed. Invalid ontology entries?")
      res.success=False
    except IndexError:
      res.trace.append("Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format")
      res.success=False
    except IOError:
      print "Error: can\'t find login file or read data"
      res.success=False
      res.trace.append("Error: can\'t find login file or read data")
    return res

  ## @brief Queries the ontology and returns the cognitive test types available
  # @param res [rapp_platform_ros_communications::userScoresForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoresForAllCategoriesSrv
  #
  # @return res [rapp_platform_ros_communications::userScoresForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoresForAllCategoriesSrv
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return testTypesList [list] The list of the available tests as they were read from the ontology
  def getTestTypesFromOntology(self,res):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_subclasses_of_topic')
    knowrob_service = rospy.ServiceProxy(serv_topic, ontologySubSuperClassesOfSrv)
    testTypesReq = ontologySubSuperClassesOfSrvRequest()
    testTypesReq.ontology_class="CognitiveTests"
    testTypesResponse = knowrob_service(testTypesReq)
    if(testTypesResponse.success!=True):
      res.trace.extend(testTypesResponse.trace)
      res.trace.append("cannot load test categories from ontology")
      res.error=testTypesResponse.error+"cannot load test categories from ontology"
      res.success=False
      return True,""
    testTypesList=[]
    for s in testTypesResponse.results:
      tmpList=s.split('#')
      testTypesList.append(tmpList[1])
    return False,testTypesList

  ## @brief Gets the users ontology alias and if it doesnt exist it creates it  
  # @param username [string] The user's username
  # @param res [rapp_platform_ros_communications::userScoresForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoresForAllCategoriesSrv
  #
  # @return res [rapp_platform_ros_communications::userScoresForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoresForAllCategoriesSrv
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return ontologyAlias [string] The user's ontology alias
  def getUserOntologyAlias(self,username,res):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')      
    knowrob_service = rospy.ServiceProxy(serv_topic, createOntologyAliasSrv)
    createOntologyAliasReq = createOntologyAliasSrvRequest()
    createOntologyAliasReq.username=username
    createOntologyAliasResponse = knowrob_service(createOntologyAliasReq)
    if(createOntologyAliasResponse.success!=True):
      res.trace.extend(createOntologyAliasResponse.trace)
      res.error=createOntologyAliasResponse.error
      res.success=False
      returnWithError=True;
      return True,""
    return False,createOntologyAliasResponse.ontology_alias

  ## @brief Calculates and returns the user's scores for the provided test types  
  # @param testTypesList [list] The list of the available tests as they were read from the ontology
  # @param userOntologyAlias [string] The user's ontology alias
  # @param upToTime [long] The time up to which the user's performance records are taken into account
  # 
  # @return testScores [list] The test scores for each category
  def calculateUserScoresForCategories(self,testTypesList,userOntologyAlias,upToTime):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests') 
    scoresPerCategory=[]
    d1=OrderedDict()
    for s in testTypesList:
      userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
      userPerformanceReq.test_type=s
      userPerformanceReq.ontology_alias=userOntologyAlias
      knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)
      userPerformanceResponse = knowrob_service(userPerformanceReq)      
      if(userPerformanceResponse.success!=True):
        scoresPerCategory.append(0)
      else:
        scoreOfCategory=0
        for i in range(len(userPerformanceResponse.tests)):
          if(int(userPerformanceResponse.timestamps[i])<upToTime):
            scoreOfCategory=scoreOfCategory+(int(userPerformanceResponse.scores[i])*int(userPerformanceResponse.difficulty[i]))
        scoresPerCategory.append(long(scoreOfCategory/len(userPerformanceResponse.tests)))
    return scoresPerCategory    
