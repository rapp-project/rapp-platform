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
  userPerformanceCognitveTestsSrv,
  userPerformanceCognitveTestsSrvRequest,
  userPerformanceCognitveTestsSrvResponse,
  cognitiveTestsOfTypeSrv,
  cognitiveTestsOfTypeSrvRequest,
  cognitiveTestsOfTypeSrvResponse,

  )
from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

## @class TestSelector
# @brief Provides the necessary functions for selecting a cognitive exercise test
#
# It implements the cognitive exercise chooser service
class TestSelector:

  ## @brief The callback function of the cognitive exercise chooser service, all other functions of the class are called from within this function
  ## @brief The cognitive exercise chooser service callback
  # @param req [rapp_platform_ros_communications::testSelectorSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception AIOError
  # @exception Exception KeyError
  # @exception Exception AppError
  def chooserFunction(self,req):
    try:      
      res = testSelectorSrvResponse()
      currentTimestamp = int(time.time())      
      #Load parameters from yaml file
      difficultyModifier1to2,difficultyModifier2to3,historyBasedOnNumOfTestsAndNotTime,pastMonths,pastTests,lookBackTimeStamp=self.loadParamDifficultyModifiersAndHistorySettings()
      #Get user ontology alias
      userOntologyAlias=CognitiveExerciseHelperFunctions.getUserOntologyAlias(req.username)      
      #Get user language
      userLanguage=CognitiveExerciseHelperFunctions.getUserLanguage(req.username)
      res.language=userLanguage
      if(not req.overwriteTestXmlFile==""):
        self.retrieveDataFromTestXml("/cognitiveTests/"+req.overwriteTestXmlFile,userLanguage,res)         
      else:
        #Get test types from ontology
        testTypesList=CognitiveExerciseHelperFunctions.getTestTypesFromOntology()
        #Determine the test type of the to be selected test
        testType=self.determineTestType(req.testType,req.testSubType,testTypesList,userOntologyAlias,res.trace)        
        #Get user performance records and determine difficulty of the to be selected test for given test type
        chosenDif,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp=self.getUserPerformanceRecordsAndDetermineTestDifficultyForTestType(testType,userOntologyAlias,currentTimestamp,lookBackTimeStamp,difficultyModifier1to2,difficultyModifier2to3,historyBasedOnNumOfTestsAndNotTime,pastTests,res.trace)
        #Get all tests of a give type from the ontology
        testsOfTypeOrdered=self.getCognitiveTestsOfType(testType,req.testSubType,userLanguage,chosenDif,res.trace)            
        #Determine the least recently used (LRU) test and retrieve the .xml test file
        testName,testFilePath=self.getLRUtestOfTypeAndXmlPath(testsOfTypeOrdered,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp)      
        #res.test=testName
        #Parse the .xml test file name and assign the data to the testSelectorSrvResponse response srv           
        self.retrieveDataFromTestXml(testFilePath,userLanguage,res)      
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
      AppError.passErrorToRosSrv(e,res) 
    return res

  ## @brief Validates the provided test type
  # @param testType [string] The test type (category)
  # @param testTypesList [list] The list of the available tests as they were read from the ontology
  # @param userOntologyAlias [string] The user's ontology alias
  # @param res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  #
  # @return trace [string] The trace argument of the service as defined in the testSelectorSrv
  # @return testType [string] The test type (category)
  # @exception Exception AppError
  def determineTestType(self,testType,testSubType,testTypesList,userOntologyAlias,trace):
    if(testType=="" and not testSubType==""):
      error="Error, testSubType provided but testType not provided"
      raise AppError(error,error)
    if(testType==""):
        testType=self.determineTestTypeIfNotProvided(testTypesList,userOntologyAlias,trace)
    else:
      if (testType not in testTypesList):
        error="testType provided does not exist"
        raise AppError(error,error)                
    return testType    

  ## @brief Determines the test type, if it was not provided in the service call as an argument. It selects the test category that was least recently used from the specific user
  # @param testTypesList [list] The list of the available tests as they were read from the ontology
  # @param userOntologyAlias [string] The user's ontology alias
  # @param res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  #
  # @return res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  # @return testType [string] The test type (category)
  def determineTestTypeIfNotProvided(self,testTypesList,userOntologyAlias,trace):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests')  
    trace.append("no test type provided, will use least recently used")
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
        tmpUserPerfOrganizedByTimestamp=CognitiveExerciseHelperFunctions.organizeUserPerformanceByTimestamp(userPerformanceResponse)
        t1=tmpUserPerfOrganizedByTimestamp.items()[0][0]
        d1[t1]=[s]
    if(len(noRecordsList)>0):
      testType=random.choice(noRecordsList)
      trace.append("made random choice from the test types which were never used by the user, choice was: "+ testType)
    else:
      d1=OrderedDict(sorted(d1.items(), key=lambda t: t[0]))
      testType=d1.values()[0][0]
      trace.append("all test types had performance records.. least recently used one was :"+ testType)
    return testType 
    
  ## @brief Organizes the test performance entries of the user by timestamp and determines the appropriate difficulty for the to be returned test
  # @param testType [string] The test type (category)
  # @param userOntologyAlias [string] The user's ontology alias
  # @param currentTimestamp [int] The timestamp at the time of the service call
  # @param lookBackTimeStamp [int] The number of months backwards in seconds as in unix timestamp
  # @param difficultyModifier1to2 [int] The first difficulty modifier
  # @param difficultyModifier2to3 [int] The second difficulty modifier
  # @param historyBasedOnNumOfTestsAndNotTime [bool] True if past performance records are based on number of tests and not time
  # @param pastTests [int] Number of tests backwards
  # @param res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  #
  # @return chosenDif [string] The chosen difficulty setting
  # @return noUserPerformanceRecordsExist [bool] True if no user performance records exit for the user for the given test type
  # @return userPerfOrganizedByTimestamp [OrderedDict] The user's performance records in a dictionary, ordered by timestamp  
  # @return res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  def getUserPerformanceRecordsAndDetermineTestDifficultyForTestType(self,testType,userOntologyAlias,currentTimestamp,lookBackTimeStamp,difficultyModifier1to2,difficultyModifier2to3,historyBasedOnNumOfTestsAndNotTime,pastTests,trace):
    noUserPerformanceRecordsExist=False;  
    userPerformanceResponse=CognitiveExerciseHelperFunctions.getUserPerformanceRecordsForTestType(testType,userOntologyAlias)     
    userPerfOrganizedByTimestamp=[]
    if(userPerformanceResponse.success!=True):
      trace.extend(userPerformanceResponse.trace)
      trace.append("KnowRob wrapper returned no performance records for this user.. will start with a difficulty setting of 1")
      noUserPerformanceRecordsExist=True
      chosenDif="1"
    else:
      userPerfOrganizedByTimestamp=CognitiveExerciseHelperFunctions.organizeUserPerformanceByTimestamp(userPerformanceResponse)
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
      trace.append("user score :"+str(userScore))    
      if(userScore==0):
        chosenDif="1"
      elif(userScore<difficultyModifier1to2):
        chosenDif="1"
      elif(userScore<difficultyModifier2to3):
        chosenDif="2"
      else:   #(userScore>0.75*3*100)
        chosenDif="3"
    trace.append("Chosen Diff :"+chosenDif)
    return chosenDif,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp
    
  ## @brief Gets the cognitive tests of the given type and difficulty available in the ontology  
  # @param testType [string] The test type (category)
  # @param testSubType [string] The test sub type
  # @param userLanguage [string] The user's language
  # @param chosenDif [string] The difficulty setting
  # @param res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  #  
  # @return testsOfTypeOrdered [dict] The cognitive tests of the given type and difficulty setting
  # @return res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  # @exception Exception AppError
  def getCognitiveTestsOfType(self,testType,testSubType,userLanguage,chosenDif,trace):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_cognitive_tests_of_type')
    cognitiveTestsOfTypeSrvReq=cognitiveTestsOfTypeSrvRequest()
    cognitiveTestsOfTypeSrvReq.test_type=testType
    cognitiveTestsOfTypeSrvReq.test_language=userLanguage
    knowrob_service = rospy.ServiceProxy(serv_topic, cognitiveTestsOfTypeSrv)
    cognitiveTestsOfTypeResponse = knowrob_service(cognitiveTestsOfTypeSrvReq)
    if(cognitiveTestsOfTypeResponse.success!=True):
      raise AppError(cognitiveTestsOfTypeResponse.error, cognitiveTestsOfTypeResponse.trace)
    testsOfTypeOrdered=self.filterTestsbyDifficulty(cognitiveTestsOfTypeResponse,chosenDif,testSubType,trace)    
    return testsOfTypeOrdered
      
  ## @brief Gets the least recently used test of the given test type and difficulty and obtains the path to the test xml file
  # @param testsOfTypeOrdered [dict] The cognitive tests of the given type and difficulty setting
  # @param noUserPerformanceRecordsExist [bool] True if no user performance records exit for the user for the given test type
  # @param userPerfOrganizedByTimestamp [OrderedDict] The user's performance records in a dictionary, ordered by timestamp  
  # 
  # @return finalTestFilePath [string] The file path of the xml file that contains the test
  # @return testName [string] The name of the test     
  # @exception Exception AppError
  def getLRUtestOfTypeAndXmlPath(self,testsOfTypeOrdered,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp):    
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
      error="Invalid test name retrieved from ontology, did not contain #"
      raise AppError(error,error)
    testName=tmpList[1]
    return testName,finalTestFilePath

  ## @brief Retrieves the questions, answers etc of the test from the xml file
  # @param testFilePath [string] The file path of the xml file that contains the test  
  # @param userLanguage [string] The language of the user
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  def retrieveDataFromTestXml(self,testFilePath,userLanguage,res):
    rospack = rospkg.RosPack()
    testFilePath=rospack.get_path('rapp_cognitive_exercise')+testFilePath
    tree = ET.parse(testFilePath)
    root = tree.getroot()
    res.testType=root.find("testType").text.encode('UTF-8')
    res.testSubType=root.find("testSubType").text.encode('UTF-8')
    res.test=root.find("name").text.encode('UTF-8')
    language=root.find('Languages')
    for question in language.find(userLanguage):
      res.questions.append(question.find("body").text.encode('UTF-8'))
      res.correctAnswers.append(question.find("correctAnswer").text.encode('UTF-8'))
      line=StringArrayMsg()
      for answers in question.findall('answer'):
        line.s.append(answers.find("body").text.encode('UTF-8'))
      res.answers.append(line)

  ## @brief Calculates the user's score from the performance entries
  # @param performanceEntries [dict] The dictionary containing the cognitive test performance entries
  #
  # @return score [int] The user's score
  def calculateUserScore(self,performanceEntries):
    score=0
    if (len(performanceEntries)>0):
      for k, v in performanceEntries.items():
        score=score+int(v[0][1])*int(v[0][2])
      score=score/len(performanceEntries)
      return score
    else:
      return 0
      
  ## @brief Filters a dictionary containing cognitive tests by keeping only those of the given difficulty  
  # @param testsOfType [dict] The dictionary containing the cognitive tests
  # @param chosenDif [string] The given difficulty
  # @param testSubType [string] The test sub type
  # @param res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  #   
  # @return res.trace [string] The trace argument of the service as defined in the testSelectorSrv
  # @exception Exception AppError
  def filterTestsbyDifficulty(self,testsOfType,chosenDif,testSubType,trace):
    success=False
    testSubTypePrefix="http://knowrob.org/kb/knowrob.owl#"
    d=dict()
    intDif=int(chosenDif)
    if(intDif==0):
      error="Error, no tests of type contained in the ontology for any difficulty, probably wrong testSubType provided or testSubType belongs to different testType."
      raise AppError(error,error)
      return d
    else:
      for i in range(len(testsOfType.tests)):
        if(testsOfType.difficulty[i]==chosenDif):
          if(not testSubType==""):
            if(testSubTypePrefix+testSubType==testsOfType.subtype[i]):
              tlist=[testsOfType.file_paths[i],testsOfType.difficulty[i],testsOfType.subtype[i]]
              d[testsOfType.tests[i]]=[tlist]
          else:
            tlist=[testsOfType.file_paths[i],testsOfType.difficulty[i],testsOfType.subtype[i]]
            d[testsOfType.tests[i]]=[tlist]
      if(not len(d)>0):
        trace.append("downscaling difficulty by 1 as no test exists for diff = " +chosenDif);
        chosenDif=str(int(chosenDif)-1)
        success,d=self.filterTestsbyDifficulty(testsOfType,chosenDif,testSubType,trace)
      else:
        success=True
      return d

  ## @brief Load the difficulty modifiers from the ros yaml file 
  #  
  # @return difficultyModifier1to2 [int] The first difficulty modifier
  # @return difficultyModifier2to3 [int] The second difficulty modifier
  # @return historyBasedOnNumOfTestsAndNotTime [bool] True if past performance records are based on number of tests and not time
  # @return pastMonths [int] Number of months backwards
  # @return pastTests [int] Number of tests backwards
  # @return lookBackTimeStamp [long] The number of months backwards in seconds as in unix timestamp
  def loadParamDifficultyModifiersAndHistorySettings(self):
    difficultyModifier1to2 = int(rospy.get_param('rapp_cognitive_test_selector_difficulty_modifier_1_from_1_to_2'))     
    difficultyModifier2to3 = int(rospy.get_param('rapp_cognitive_test_selector_difficulty_modifier_2_from_2_to_3'))          
    historyBasedOnNumOfTestsAndNotTime=False
    historyBasedOnNumOfTestsAndNotTime = (str(rospy.get_param('rapp_cognitive_test_selector_past_performance_based_on_number_of_tests_and_not_time'))).lower()
    if(historyBasedOnNumOfTestsAndNotTime=="true"):
      historyBasedOnNumOfTestsAndNotTime=True     
    pastMonths = int(rospy.get_param('rapp_cognitive_test_selector_past_performance_number_of_past_months'))    
    pastTests = int(rospy.get_param('rapp_cognitive_test_selector_past_performance_number_of_past_tests'))    
    lookBackTimeStamp=pastMonths*30*24*3600
    return difficultyModifier1to2,difficultyModifier2to3,historyBasedOnNumOfTestsAndNotTime,pastMonths,pastTests,lookBackTimeStamp
