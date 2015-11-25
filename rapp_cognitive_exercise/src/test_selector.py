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

class TestSelector:      

  def chooserFunction(self,req):
    currentTimestamp = int(time.time()) 
    lookBackTimeStamp=15552000000 #15552000000 for last 3 months
    try:
      res = testSelectorSrvResponse()
        
      returnWithError,userOntologyAlias=self.getUserOntologyAlias(req.username,res)
      if(returnWithError):
        return res

      returnWithError,userLanguage=self.getUserLanguage(req.username,res)
      if(returnWithError):
        return res

      returnWithError,testTypesList=self.getTestTypesFromOntology(res)
      if(returnWithError):
        return res
        
      if(req.testType==""):
        testType=self.determineTestTypeIfNotProvided(res,testTypesList,userOntologyAlias)
      else:
        if (req.testType not in testTypesList):
          res.trace.append("testType provided does not exist")
          res.error="testType provided does not exist"
          res.success=False
          return res
        else:
          testType=req.testType
      
      chosenDif,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp=self.organizeUserPerformanceByTimestampAndDetermineTestDifficulty(testType,userOntologyAlias,currentTimestamp,lookBackTimeStamp,res)
            
      returnWithError,testsOfTypeOrdered=self.getCognitiveTestsOfTypeAndDifficultyOrderedByTimestamp(testType,userLanguage,chosenDif,res,returnWithError)      
      if(returnWithError):
        return res      

      finalTestFilePath,finalTestname=self.getLRUtestOfTypeAndXmlPath(testsOfTypeOrdered,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp)
      
      #Retrieve the name of the selected test
      tmpList=finalTestname.split('#')
      res.test=tmpList[1]

      #Parse the test xml file and retrieve the desired information
      rospack = rospkg.RosPack()
      localPackagePath=rospack.get_path('rapp_cognitive_exercise')
      finalTestFilePath=localPackagePath+finalTestFilePath
      res.trace.append(finalTestFilePath)
      self.retrieveDataFromTestXml(finalTestFilePath,res,userLanguage)
      res.language=userLanguage
      res.success=True

    except IndexError:
      res.trace.append("Null pointer exception.. some argument was empty")
      res.error="Null pointer exception.. some argument was empty"
      res.success=False
    except IOError:
      res.success=False
      res.trace.append("IO Error, cant open file or read data")
      res.error="IO Error, cant open file or read data"
    return res

  def getUserLanguage(self,username,res):
    #get user language
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_fetch_data_topic')	
    knowrob_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    fetchDataSrvReq = fetchDataSrvRequest()
    fetchDataSrvReq.req_cols=["language"]
    fetchDataSrvReq.where_data=[StringArrayMsg(s=["username",username])]
    fetchDataSrvResponse = knowrob_service(fetchDataSrvReq)
    if(fetchDataSrvResponse.success.data!=True):      
      res.trace.extend(fetchDataSrvResponse.trace)
      res.error=fetchDataSrvResponse.trace[0]
      res.success=False
      return True,""
    return False,fetchDataSrvResponse.res_data[0].s[0]

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
    
  def organizeUserPerformanceByTimestampAndDetermineTestDifficulty(self,testType,userOntologyAlias,currentTimestamp,lookBackTimeStamp,res):
    # check if performance records exist for the selected test in order to determine the difficulty setting
    noUserPerformanceRecordsExist=False;
    chosenDif="1"
    userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
    userPerformanceReq.test_type=testType
    userPerformanceReq.ontology_alias=userOntologyAlias
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests')
    knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)
    userPerformanceResponse = knowrob_service(userPerformanceReq)
    
    userPerfOrganizedByTimestamp=[]
    if(userPerformanceResponse.success!=True):
      res.trace.extend(userPerformanceResponse.trace)
      res.trace.append("KnowRob wrapper returned no performance records for this user.. will start with a a difficulty setting of 1")
      noUserPerformanceRecordsExist=True
    else:
      userPerfOrganizedByTimestamp=self.organizeUserPerformance(userPerformanceResponse)
      for k, v in userPerfOrganizedByTimestamp.items():
        if(currentTimestamp-k>lookBackTimeStamp):
          del userPerfOrganizedByTimestamp[k]
        else:
          break

      userScore=self.calculateUserScore(userPerfOrganizedByTimestamp)
      res.trace.append("user score :"+str(userScore))
      if(userScore==0):
        chosenDif="1"
      elif(userScore<0.75*100):
        chosenDif="1"
      elif(userScore<0.75*2*100):
        chosenDif="2"
      else:   #(userScore>0.75*3*100)
        chosenDif="3"
    res.trace.append("Chosen Diff :"+chosenDif)
    return chosenDif,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp
    
    
  def getCognitiveTestsOfTypeAndDifficultyOrderedByTimestamp(self,testType,userLanguage,chosenDif,res,returnWithError):
    #Get cognitive tests of the selected type.
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_cognitive_tests_of_type')
    cognitiveTestsOfTypeSrvReq=cognitiveTestsOfTypeSrvRequest()
    cognitiveTestsOfTypeSrvReq.test_type=testType
    cognitiveTestsOfTypeSrvReq.test_language=userLanguage
    knowrob_service = rospy.ServiceProxy(serv_topic, cognitiveTestsOfTypeSrv)
    cognitiveTestsOfTypeResponse = knowrob_service(cognitiveTestsOfTypeSrvReq)

    if(cognitiveTestsOfTypeResponse.success!=True):
      res.trace.extend(cognitiveTestsOfTypeResponse.trace)
      res.error=cognitiveTestsOfTypeResponse.error
      res.success=False
      return True,""

    success,testsOfTypeOrdered=self.filterTestsbyDifficulty(cognitiveTestsOfTypeResponse,chosenDif,res)

    #If no test exists
    if(not success):
      res.trace.append("Error, no tests of type contained in the ontology... cannot proceed")
      res.error="Error, no tests of type contained in the ontology... cannot proceed"
      res.success=False
      return True,""
    return False,testsOfTypeOrdered
      
  def getLRUtestOfTypeAndXmlPath(self,testsOfTypeOrdered,noUserPerformanceRecordsExist,userPerfOrganizedByTimestamp):
    #Choose the least recently used test of the given test type and difficulty and obtain the path to the xml file
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
    return finalTestFilePath,finalTestname


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

  def organizeUserPerformance(self,userPerf):
    d=OrderedDict()
    for i in range(len(userPerf.tests)):
      tlist=[userPerf.tests[i],userPerf.scores[i],userPerf.difficulty[i]]
      d[int(userPerf.timestamps[i])]=[tlist]
    d=OrderedDict(sorted(d.items(), key=lambda t: t[0], reverse=True))
    return d

  def calculateUserScore(self,d):
    score=0
    if (len(d)>0):
      for k, v in d.items():
        score=score+int(v[0][1])*int(v[0][2])
      score=score/len(d)
      return score
    else:
      return 0

  def filterTestsbyDifficulty(self,testsOfType,chosenDif,res):
    success=False
    d=dict()
    intDif=int(chosenDif)
    if(intDif==0):
      return success,d
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
      return success,d

  def getUserOntologyAlias(self,username,res):
    #obtain user's ontology alias. It will be created if it does not exist in case of invalid username it will be caught
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



