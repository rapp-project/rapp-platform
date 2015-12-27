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
  userPerformanceCognitveTestsSrv,
  userPerformanceCognitveTestsSrvRequest,
  userScoreHistoryForAllCategoriesSrv,
  userScoreHistoryForAllCategoriesSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  CognitiveExercisePerformanceRecordsMsg,
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
  # @exception Exception IndexError
  # @exception Exception AIOError
  # @exception Exception ValueError
  def returnUserHistory(self,req):
    try:
      res = userScoreHistoryForAllCategoriesSrvResponse()
      
      returnWithError,userOntologyAlias=self.getUserOntologyAlias(req.username,res)
      if(returnWithError):
        return res      

      returnWithError,fromTime,toTime=self.validateTimeRange(req.fromTime,req.toTime,res)
      if(returnWithError):
        return res              

      returnWithError,testTypesList=self.getTestTypesFromOntology(res)
      if(returnWithError):
        return res
      
      if(not req.testType==""):
        if(req.testType not in testTypesList):
          res.success=False
          res.error="invalid test type, not contained in ontology subclasses of cognitive test types"
          res.trace.append("invalid test type, not contained in ontology subclasses of cognitive test types")
          return res
        testTypesList=[]
        testTypesList.append(req.testType)
        
      self.retrieveTestHistoryForTestCategories(testTypesList,userOntologyAlias,fromTime,toTime,res)  
      res.success=True    

    except ValueError:
      res.trace.append("Value Error, probably conversion from integer to string failed. Invalid ontology entries?")
      res.success=False
    except IndexError:
      res.trace.append("Null pointer exception")
      res.success=False
    except IOError:
      print "Error: can\'t find login file or read data"
      res.success=False
      res.trace.append("Error: can\'t find login file or read data")
    return res

  ## @brief Queries the ontology and returns the cognitive test types available
  # @param res [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoreHistoryForAllCategoriesSrv
  #
  # @return res [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoreHistoryForAllCategoriesSrv
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
  # @param res [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoreHistoryForAllCategoriesSrv
  #
  # @return res [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoreHistoryForAllCategoriesSrv
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

  ## @brief Retrieves the test history of the user for the provided test categories  
  # @param testTypesList [list] The list of the available tests as they were read from the ontology
  # @param userOntologyAlias [string] The user's ontology alias
  # @param upToTime [long] The time up to which the user's performance records are taken into account
  # 
  # @return res [rapp_platform_ros_communications::userScoreHistoryForAllCategoriesSrvResponse::Response&] The output arguments of the service as defined in the userScoreHistoryForAllCategoriesSrv
  def retrieveTestHistoryForTestCategories(self,testTypesList,userOntologyAlias,fromTime,toTime,res):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests') 
    scoresPerCategory=[]
    d1=OrderedDict()
    for s in testTypesList:
      userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
      userPerformanceReq.test_type=s
      userPerformanceReq.ontology_alias=userOntologyAlias
      knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)
      userPerformanceResponse = knowrob_service(userPerformanceReq)      
      if(userPerformanceResponse.success==True): 
        userPerfOrganizedByTimestamp=self.organizeUserPerformance(userPerformanceResponse)
        sumCategoryScore=0
        sumCategoryScoreDivideBy=0;
        countValidTests=0;
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
            res.records.append(tmpRecord)


  ## @brief Validates the fromTime and toTime variables  
  # @param fromTime [long] The time from which the performance records are evaluated
  # @param toTime [long] The time up to which the performance records are evaluated
  # 
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report  
  # @return fromTime [long] The time from which the performance records are evaluated
  # @return toTime [long] The time up to which the performance records are evaluated
  def validateTimeRange(self,fromTime,toTime,res):
    if (toTime is None or toTime==0):
      toTime=9999999999999999999999
    if (fromTime is None or fromTime==0):
      fromTime=0    
    if(fromTime>=toTime):
      res.success=False
      res.error="Invalid time range, fromTime is later than toTime"
      return True,0,0      
    return False,fromTime,toTime

  ## @brief Organizes the user's performance entries by timestamp
  # @param d [dict] The dictionary containing the user's performance entries
  #
  # @return d [OrderedDict] The dictionary containing the user's performance entries organized by timestamp
  def organizeUserPerformance(self,userPerf):
    d=OrderedDict()
    for i in range(len(userPerf.tests)):
      tlist=[userPerf.tests[i],userPerf.scores[i],userPerf.difficulty[i],userPerf.subtypes[i]]
      d[int(userPerf.timestamps[i])]=[tlist]
    d=OrderedDict(sorted(d.items(), key=lambda t: t[0], reverse=True))
    return d
