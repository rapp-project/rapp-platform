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

import rospkg
import rospy
import sys
import xml.etree.ElementTree as ET
import calendar
import time
import random
from datetime import datetime
from os.path import expanduser
from collections import OrderedDict
from random import randint
from codecs import open
from app_error_exception import AppError
from helper_functions import CognitiveExerciseHelperFunctions
from rapp_platform_ros_communications.srv import (
  ontologySubSuperClassesOfSrv,
  ontologySubSuperClassesOfSrvRequest,
  ontologySubSuperClassesOfSrvResponse,
  testSelectorSrv,
  testSelectorSrvResponse,
  createOntologyAliasSrv,
  createOntologyAliasSrvRequest,
  createOntologyAliasSrvResponse,
  userPerformanceCognitveTestsSrv,
  userPerformanceCognitveTestsSrvRequest,
  userPerformanceCognitveTestsSrvResponse,
  cognitiveTestsOfTypeSrv,
  cognitiveTestsOfTypeSrvRequest,
  cognitiveTestsOfTypeSrvResponse,
  fetchDataSrv,
  fetchDataSrvRequest,
  fetchDataSrvResponse
  )
from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

## @class TestSelector
# @brief Provides the necessary functions for selecting a cognitive exercise test
#
# It implements the cognitive exercise chooser service
class TestSelector:
  ##This variable refers to the length of time backwards that the user's cognitive test performance records will be queried, default value is 15552000000 -3 months
  #lookBackTimeStamp=15552000000 #15552000000 for last 3 months    

  ## @brief The callback function of the cognitive exercise chooser service, all other functions of the class are called from within this function
  ## @brief The cognitive exercise chooser service callback
  # @param req [rapp_platform_ros_communications::testSelectorSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception AIOError
  def chooserFunction(self,req):
    try:
      rospack = rospkg.RosPack()
      res = testSelectorSrvResponse()
      currentTimestamp = int(time.time()) 
      modifier1,modifier2,historyBasedOnNumOfTestsAndNotTime,pastMonths,pastTests,lookBackTimeStamp=self.loadParamDifficultyModifiersAndHistorySettings(res)
      userOntologyAlias=CognitiveExerciseHelperFunctions.getUserOntologyAlias(req.username)
      userLanguage=self.getUserLanguage(req.username,res)
      testTypesList=self.getTestTypesFromOntology()
      testType=self.determineTestType(req.testType,testTypesList,userOntologyAlias,res)      
      chosenDif,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp=self.organizeUserPerformanceByTimestampAndDetermineTestDifficulty(testType,userOntologyAlias,currentTimestamp,lookBackTimeStamp,res,modifier1,modifier2,historyBasedOnNumOfTestsAndNotTime,pastTests)
      testsOfTypeOrdered=self.getCognitiveTestsOfType(testType,userLanguage,chosenDif,res)            
      finalTestFilePath,finalTestname=self.getLRUtestOfTypeAndXmlPath(testsOfTypeOrdered,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp,res)
      finalTestFilePath=rospack.get_path('rapp_cognitive_exercise')+finalTestFilePath
      #res.trace.append(finalTestFilePath)
      self.retrieveDataFromTestXml(finalTestFilePath,res,userLanguage)      
      res.success=True
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.error="IndexError: "+str(e)
      res.success=False
    except IOError, e:
      res.success=False
      res.trace.append("IOError: "+str(e))
      res.error="IOError: "+str(e)
    except KeyError, e:
      res.success=False
      res.trace.append('"KeyError (probably invalid cfg/.yaml parameter) "%s"' % str(e))
      res.error='"KeyError (probably invalid cfg/.yaml parameter) "%s"' % str(e)
    except AppError as e:
      res.success=False
      if e.args[0] is not None and type(e.args[0]) is str:
        res.error=e.args[0]
      else:
        res.error="Unspecified application Error.."
      if (e.args[1] is not None and type(e.args[1]) is list):
        res.trace.extend(e.args[1])   
      elif(e.args[1] is not None and type(e.args[1]) is str):
        res.trace.append(e.args[1])
    return res

  def determineTestType(self,testType,testTypesList,userOntologyAlias,res):
    if(testType==""):
        testType=self.determineTestTypeIfNotProvided(res,testTypesList,userOntologyAlias)
    else:
      if (testType not in testTypesList):
        error="testType provided does not exist"
        raise AppError(error,error)                
    return testType    

  ## @brief Queries the MySQL database through the MySQL wrapper and returns the user's language
  # @param username [string] The username of the user as is in the MySQL database
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return userLanguage [string] The user's language setting
  def getUserLanguage(self,username,res):
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_fetch_data_topic')	
    knowrob_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    fetchDataSrvReq = fetchDataSrvRequest()
    fetchDataSrvReq.req_cols=["language"]
    fetchDataSrvReq.where_data=[StringArrayMsg(s=["username",username])]
    fetchDataSrvResponse = knowrob_service(fetchDataSrvReq)
    if(fetchDataSrvResponse.success.data!=True): 
      raise AppError(fetchDataSrvResponse.trace[0], fetchDataSrvResponse.trace)      
    res.language=fetchDataSrvResponse.res_data[0].s[0]
    return fetchDataSrvResponse.res_data[0].s[0]

  ## @brief Queries the ontology and returns the cognitive test types available
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return testTypesList [list] The list of the available tests as they were read from the ontology
  def getTestTypesFromOntology(self):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_subclasses_of_topic')
    knowrob_service = rospy.ServiceProxy(serv_topic, ontologySubSuperClassesOfSrv)
    testTypesReq = ontologySubSuperClassesOfSrvRequest()
    testTypesReq.ontology_class="CognitiveTests"
    testTypesResponse = knowrob_service(testTypesReq)
    if(testTypesResponse.success!=True):     
      testTypesResponse.trace.append("cannot load test categories from ontology")
      raise AppError(testTypesResponse.error+"cannot load test categories from ontology",testTypesResponse.trace)      
    testTypesList=[]
    for s in testTypesResponse.results:
      tmpList=s.split('#')
      testTypesList.append(tmpList[1])
    return testTypesList
    
  ## @brief Determines the test type, if it was not provided in the service call as an argument. It selects the test category that was least recently used from the specific user
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @param testTypesList [list] The list of the available tests as they were read from the ontology
  # @param userOntologyAlias [string] The user's ontology alias
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return testType [string] The test type (category)
  def determineTestTypeIfNotProvided(self,res,testTypesList,userOntologyAlias):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests')  
    res.trace.append("no test type provided, will use least recently used")
    noRecordsList=[]
    d1=OrderedDict()
    for s in testTypesList:
      userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
      userPerformanceReq.test_type=s
      userPerformanceReq.ontology_alias=userOntologyAlias
      knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)
      userPerformanceResponse = knowrob_service(userPerformanceReq)
      if(userPerformanceResponse.success!=True):
        noRecordsList.append(s)
      else:
        tmpUserPerfOrganizedByTimestamp=self.organizeUserPerformance(userPerformanceResponse)
        t1=tmpUserPerfOrganizedByTimestamp.items()[0][0]
        d1[t1]=[s]
    if(len(noRecordsList)>0):
      testType=random.choice(noRecordsList)
      res.trace.append("made random choice from the test types which were never used by the user, choice was: "+ testType)
    else:
      d1=OrderedDict(sorted(d1.items(), key=lambda t: t[0]))
      testType=d1.values()[0][0]
      res.trace.append("all test types had performance records.. least recently used one was :"+ testType)
    return testType 
    
  ## @brief Organizes the test performance entries of the user by timestamp and determines the appropriate difficulty for the to be returned test
  # @param testType [string] The test type (category)
  # @param userOntologyAlias [string] The user's ontology alias
  # @param currentTimestamp [int] The timestamp at the time of the service call
  # @param lookBackTimeStamp [int] The number of months backwards in seconds as in unix timestamp
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @param modifier1 [int] The first difficulty modifier
  # @param modifier2 [int] The second difficulty modifier
  # @param historyBasedOnNumOfTestsAndNotTime [bool] True if past performance records are based on number of tests and not time
  # @param pastTests [int] Number of tests backwards
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return chosenDif [string] The chosen difficulty setting
  # @return noUserPerformanceRecordsExist [bool] True if no user performance records exit for the user for the given test type
  # @return userPerfOrganizedByTimestamp [OrderedDict] The user's performance records in a dictionary, ordered by timestamp  
  def organizeUserPerformanceByTimestampAndDetermineTestDifficulty(self,testType,userOntologyAlias,currentTimestamp,lookBackTimeStamp,res,modifier1,modifier2,historyBasedOnNumOfTestsAndNotTime,pastTests):
    noUserPerformanceRecordsExist=False;    
    userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
    userPerformanceReq.test_type=testType
    userPerformanceReq.ontology_alias=userOntologyAlias
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests')
    knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)
    userPerformanceResponse = knowrob_service(userPerformanceReq)    
    userPerfOrganizedByTimestamp=[]
    if(userPerformanceResponse.success!=True):
      res.trace.extend(userPerformanceResponse.trace)
      res.trace.append("KnowRob wrapper returned no performance records for this user.. will start with a difficulty setting of 1")
      noUserPerformanceRecordsExist=True
    else:
      userPerfOrganizedByTimestamp=self.organizeUserPerformance(userPerformanceResponse)
      if(historyBasedOnNumOfTestsAndNotTime):
        if(len(userPerfOrganizedByTimestamp)>pastTests):          
          counter=0;
          for k, v in userPerfOrganizedByTimestamp.items():
            counter=counter+1
            if(counter>pastTests):
              del userPerfOrganizedByTimestamp[k]       
      else:
        for k, v in userPerfOrganizedByTimestamp.items():
          if(currentTimestamp-k>lookBackTimeStamp):
            del userPerfOrganizedByTimestamp[k]
          else:
            break

      userScore=self.calculateUserScore(userPerfOrganizedByTimestamp)
      res.trace.append("user score :"+str(userScore))
      
      if(userScore==0):
        chosenDif="1"
      elif(userScore<modifier1):
        chosenDif="1"
      elif(userScore<modifier2):
        chosenDif="2"
      else:   #(userScore>0.75*3*100)
        chosenDif="3"
    res.trace.append("Chosen Diff :"+chosenDif)
    return chosenDif,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp
    
  ## @brief Gets the cognitive tests of the given type and difficulty available in the ontology  
  # @param testType [string] The test type (category)
  # @param userLanguage [string] The user's language
  # @param chosenDif [string] The difficulty setting
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return testsOfTypeOrdered [dict] The cognitive tests of the given type and difficulty setting
  def getCognitiveTestsOfType(self,testType,userLanguage,chosenDif,res):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_cognitive_tests_of_type')
    cognitiveTestsOfTypeSrvReq=cognitiveTestsOfTypeSrvRequest()
    cognitiveTestsOfTypeSrvReq.test_type=testType
    cognitiveTestsOfTypeSrvReq.test_language=userLanguage
    knowrob_service = rospy.ServiceProxy(serv_topic, cognitiveTestsOfTypeSrv)
    cognitiveTestsOfTypeResponse = knowrob_service(cognitiveTestsOfTypeSrvReq)
    if(cognitiveTestsOfTypeResponse.success!=True):
      raise AppError(cognitiveTestsOfTypeResponse.error, cognitiveTestsOfTypeResponse.trace)
    testsOfTypeOrdered=self.filterTestsbyDifficulty(cognitiveTestsOfTypeResponse,chosenDif,res)    
    return testsOfTypeOrdered
      
  ## @brief Gets the least recently used test of the given test type and difficulty and obtains the path to the test xml file
  # @param testsOfTypeOrdered [dict] The cognitive tests of the given type and difficulty setting
  # @param noUserPerformanceRecordsExist [bool] True if no user performance records exit for the user for the given test type
  # @param userPerfOrganizedByTimestamp [OrderedDict] The user's performance records in a dictionary, ordered by timestamp  
  # 
  # @return finalTestFilePath [string] The file path of the xml file that contains the test
  # @return finalTestname [string] The name of the test     
  def getLRUtestOfTypeAndXmlPath(self,testsOfTypeOrdered,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp,res):    
    finalTestname=""
    finalTestFilePath=""
    if(noUserPerformanceRecordsExist):
      finalTestname=random.choice(testsOfTypeOrdered.keys())
      finalTest=testsOfTypeOrdered[finalTestname]
      finalTestFilePath=finalTest[0][0]
    else:
      testsOfTypeOrderedCopy=testsOfTypeOrdered.copy()
      for k, v in userPerfOrganizedByTimestamp.items():
        if(v[0][0] in testsOfTypeOrderedCopy):
          del testsOfTypeOrderedCopy[v[0][0]]
      if(len(testsOfTypeOrderedCopy)>0):
        finalTestname=random.choice(testsOfTypeOrderedCopy.keys())
        finalTest=testsOfTypeOrderedCopy[finalTestname]
        finalTestFilePath=finalTest[0][0]
      else:
        finalTestname=userPerfOrganizedByTimestamp.values()[len(userPerfOrganizedByTimestamp)-1]
        finalTestname=finalTestname[0][0]
        finalTest=testsOfTypeOrdered[finalTestname]
        finalTestFilePath=finalTest[0][0]    
    tmpList=finalTestname.split('#') #Retrieve the name of the selected test
    if (tmpList[1] is None):
      res.error="Invalid test name retrieved from ontology, did not contain #"
      raise AppError(error,error)       
    finalTestname=tmpList[1]
    res.test=finalTestname
    return finalTestFilePath,finalTestname

  ## @brief Retrieves the questions, answers etc of the test from the xml file
  # @param finalTestFilePath [string] The file path of the xml file that contains the test  
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @param userLanguage [string] The language of the user
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  def retrieveDataFromTestXml(self,finalTestFilePath,res,userLanguage):
    tree = ET.parse(finalTestFilePath)
    root = tree.getroot()
    res.testType=root.find("testType").text.encode('UTF-8')
    res.testSubType=root.find("testSubType").text.encode('UTF-8')
    language=root.find('Languages')
    for question in language.find(userLanguage):
      res.questions.append(question.find("body").text.encode('UTF-8'))
      res.correctAnswers.append(question.find("correctAnswer").text.encode('UTF-8'))
      line=StringArrayMsg()
      for answers in question.findall('answer'):
        line.s.append(answers.find("body").text.encode('UTF-8'))
      res.answers.append(line)

  ## @brief Organizes the user's performance entries by timestamp
  # @param d [dict] The dictionary containing the user's performance entries
  #
  # @return d [OrderedDict] The dictionary containing the user's performance entries organized by timestamp
  def organizeUserPerformance(self,userPerf):
    d=OrderedDict()
    for i in range(len(userPerf.tests)):
      tlist=[userPerf.tests[i],userPerf.scores[i],userPerf.difficulty[i]]
      d[int(userPerf.timestamps[i])]=[tlist]      
    d=OrderedDict(sorted(d.items(), key=lambda t: t[0], reverse=True)) #Order is descending    
    return d

  ## @brief Calculates the user's score from the performance entries
  # @param d [dict] The dictionary containing the cognitive test performance entries
  #
  # @return score [int] The user's score
  def calculateUserScore(self,d):
    score=0
    if (len(d)>0):
      for k, v in d.items():
        score=score+int(v[0][1])*int(v[0][2])
      score=score/len(d)
      return score
    else:
      return 0
      
  ## @brief Filters a dictionary containing cognitive tests by keeping only those of the given difficulty  
  # @param testsOfType [dict] The dictionary containing the cognitive tests
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return ontologyAlias [string] The user's ontology alias
  def filterTestsbyDifficulty(self,testsOfType,chosenDif,res):
    success=False
    d=dict()
    intDif=int(chosenDif)
    if(intDif==0):
      error="Error, no tests of type contained in the ontology... cannot proceed"
      raise AppError(error,error)
      return d
    else:
      for i in range(len(testsOfType.tests)):
        if(testsOfType.difficulty[i]==chosenDif):
          tlist=[testsOfType.file_paths[i],testsOfType.difficulty[i],testsOfType.subtype[i]]
          d[testsOfType.tests[i]]=[tlist]
      if(not len(d)>0):
        res.trace.append("downscaling difficulty by 1 as no test exists for diff = " +chosenDif);
        chosenDif=str(int(chosenDif)-1)
        success,d=self.filterTestsbyDifficulty(testsOfType,chosenDif,res)
      else:
        success=True
      return d

  ## @brief Load the difficulty modifiers from the ros yaml file  
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return modifier1 [int] The first difficulty modifier
  # @return modifier2 [int] The second difficulty modifier
  # @return historyBasedOnNumOfTestsAndNotTime [bool] True if past performance records are based on number of tests and not time
  # @return pastMonths [int] Number of months backwards
  # @return pastTests [int] Number of tests backwards
  # @return lookBackTimeStamp [long] The number of months backwards in seconds as in unix timestamp
  def loadParamDifficultyModifiersAndHistorySettings(self,res):
    modifier1 = int(rospy.get_param('rapp_cognitive_test_selector_difficulty_modifier_1_from_1_to_2'))     
    modifier2 = int(rospy.get_param('rapp_cognitive_test_selector_difficulty_modifier_2_from_2_to_3'))          
    historyBasedOnNumOfTestsAndNotTime=False
    historyBasedOnNumOfTestsAndNotTime = (str(rospy.get_param('rapp_cognitive_test_selector_past_performance_based_on_number_of_tests_and_not_time'))).lower()
    if(historyBasedOnNumOfTestsAndNotTime=="true"):
      historyBasedOnNumOfTestsAndNotTime=True     
    pastMonths = int(rospy.get_param('rapp_cognitive_test_selector_past_performance_number_of_past_months'))    
    pastTests = int(rospy.get_param('rapp_cognitive_test_selector_past_performance_number_of_past_tests'))    
    lookBackTimeStamp=pastMonths*30*24*3600
    return modifier1,modifier2,historyBasedOnNumOfTestsAndNotTime,pastMonths,pastTests,lookBackTimeStamp
