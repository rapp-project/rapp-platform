#!/usr/bin/env python

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

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
  cognitiveTestsOfTypeSrvResponse
  )
  
from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  ) 

class TestSelector:

  def chooserFunction(self,req):
    currentTimestamp = int(time.time()) #15552000000 for last 3 months
    
    try:
      res = testSelectorSrvResponse() 
      #obtain user's ontology alias. It will be created if it does not exist in case of invalid username it will be caught
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')
      if(not serv_topic):
        rospy.logerror("mysql_wrapper_rapp_read_data topic param not found")
        res.trace.append("mysql_wrapper_rapp_read_data topic param not found")
        res.error="mysql_wrapper_rapp_read_data topic param not found"
        res.success=False
        return res
      rospy.wait_for_service(serv_topic)
      knowrob_service = rospy.ServiceProxy(serv_topic, createOntologyAliasSrv)
      createOntologyAliasReq = createOntologyAliasSrvRequest()
      createOntologyAliasReq.username=req.username    
      createOntologyAliasResponse = knowrob_service(createOntologyAliasReq)
      if(createOntologyAliasResponse.success!=True):
        res.trace.extend(createOntologyAliasResponse.trace)
        res.error=createOntologyAliasResponse.error
        res.success=False
        return res     
      
      #Check if test type exists (if a test type is provided)
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_subclasses_of_topic')
      if(not serv_topic):
        rospy.logerror("rapp_knowrob_wrapper_subclasses_of_topic topic param not found")
        res.trace.append("rapp_knowrob_wrapper_subclasses_of_topic topic param not found")
        res.error="rapp_knowrob_wrapper_subclasses_of_topic topic param not found"
        res.success=False
        return res
      rospy.wait_for_service(serv_topic)
      knowrob_service = rospy.ServiceProxy(serv_topic, ontologySubSuperClassesOfSrv)
      testTypesReq = ontologySubSuperClassesOfSrvRequest()
      testTypesReq.ontology_class="CognitiveTests" 
      testTypesResponse = knowrob_service(testTypesReq)
      if(testTypesResponse.success!=True):
        res.trace.extend(testTypesResponse.trace)
        res.trace.append("cannot load test categories from ontology")
        res.error=testTypesResponse.error+"cannot load test categories from ontology"
        res.success=False
        return res
      testTypesList=[]
      for s in testTypesResponse.results:
        #res.trace.append(s)
        tmpList=s.split('#')
        testTypesList.append(tmpList[1])       
        
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests')
      if(not serv_topic):
        rospy.logerror("rapp_knowrob_wrapper_user_performance_cognitve_tests topic not foud")
        res.trace.append("mysql_wrapper_rapp_read_data topic param not found")
        res.error="mysql_wrapper_rapp_read_data topic param not found"
        res.success=False
        return res
        
      if(req.testType==""):
        res.trace.append("no test type provided, will use least recently used")
        noRecordsList=[]
        d1=OrderedDict()
        for s in testTypesList:          
          userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
          userPerformanceReq.test_type=s
          userPerformanceReq.ontology_alias=createOntologyAliasResponse.ontology_alias
          knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)       
          userPerformanceResponse = knowrob_service(userPerformanceReq)
          if(userPerformanceResponse.success!=True): 
            noRecordsList.append(s)
          else:
            tmpUserPerfOrganizedByTimestamp=self.organizeUserPerformance(userPerformanceResponse)
            t1=tmpUserPerfOrganizedByTimestamp.items()[0][0]
            d1[t1]=[s]
            
        if(len(noRecordsList)>0):
          req.testType=random.choice(noRecordsList)
          res.trace.append("made random choice from the test types which were never used by the user, choice was: "+ req.testType)
        else:         
          d1=OrderedDict(sorted(d1.items(), key=lambda t: t[0]))
          req.testType=d1.values()[0][0] 
          res.trace.append("all test types had performance records.. least recently used one was :"+ req.testType)         
      else:
        #res.trace.append("test is "+req.testType)
        if (req.testType not in testTypesList): 
          res.trace.append("testType provided does not exist")
          res.error="testType provided does not exist"
          res.success=False
          return res
      
      #testTypes=testTypesResponse.results
      #if(req.testType!=""):
        #if any(req.testType in s for s in testTypes):
          #print "something"
                

     
      noUserPerformanceRecordsExist=False;
      chosenDif="1"     
       
      userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
      userPerformanceReq.test_type=req.testType
      userPerformanceReq.ontology_alias=createOntologyAliasResponse.ontology_alias
      knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)       
      userPerformanceResponse = knowrob_service(userPerformanceReq)
      if(userPerformanceResponse.success!=True):
        #means no performance records exist. so you choose random test, test1, variation1
        res.trace.extend(userPerformanceResponse.trace)
        res.trace.append("KnowRob wrapper returned no performance records for this user.. will start with a a difficulty setting of 1")
        noUserPerformanceRecordsExist=True
        #userPerfOrganizedByTimestamp=OrderedDict() #empty
        #res.error=userPerformanceResponse.error
        #res.success=False
        #return res          
      else:          
        userPerfOrganizedByTimestamp=self.organizeUserPerformance(userPerformanceResponse)
        for k, v in userPerfOrganizedByTimestamp.items():
          if(currentTimestamp-k<15552000000):
            del userPerfOrganizedByTimestamp[k]
          else:
            break
          #else:
            #res.trace.append(str(k))                    
            #res.trace.append(v[0][1])
            #res.trace.append(v[0][2])
          #res.trace.append(v[2])              
        userScore=self.calculateUserScore(userPerfOrganizedByTimestamp)
        res.trace.append("score "+str(userScore))
        #chosenDif="1"
        if(userScore==0):
          chosenDif="1"
        elif(userScore>0.75*100):
          chosenDif="1"
        elif(userScore>0.75*2*100):
          chosenDif="2"
        elif(userScore>0.75*3*100):
          chosenDif="3"
        
      res.trace.append("Chosen Diff "+chosenDif)
      
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_cognitive_tests_of_type')
      if(not serv_topic):
        rospy.logerror("rapp_knowrob_wrapper_cognitive_tests_of_type not found")
        res.trace.append("rapp_knowrob_wrapper_cognitive_tests_of_type not found")
        res.error="rapp_knowrob_wrapper_cognitive_tests_of_type not found"
        res.success=False
        return res
      cognitiveTestsOfTypeSrvReq=cognitiveTestsOfTypeSrvRequest()
      cognitiveTestsOfTypeSrvReq.test_type=req.testType
      knowrob_service = rospy.ServiceProxy(serv_topic, cognitiveTestsOfTypeSrv)       
      cognitiveTestsOfTypeResponse = knowrob_service(cognitiveTestsOfTypeSrvReq)

      if(cognitiveTestsOfTypeResponse.success!=True):     
        res.trace.extend(cognitiveTestsOfTypeResponse.trace)
        res.error=cognitiveTestsOfTypeResponse.error
        res.success=False
        return res
      #res.trace.append("length "+str(len(cognitiveTestsOfTypeResponse.tests)))

      testsOfTypeOrdered=self.organizeTestsOfType(cognitiveTestsOfTypeResponse)
      
      if(not len(testsOfTypeOrdered)>0):
        res.trace.append("Error, no tests of type contained in the ontology... cannot proceed")
        res.error="Error, no tests of type contained in the ontology... cannot proceed"
        res.success=False
        return res
      
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
          #res.trace.append("was empty")
          finalTestname=userPerfOrganizedByTimestamp.values()[len(userPerfOrganizedByTimestamp)-1]
          #res.trace.append("Selected test name: "+finalTest[0][0])
          finalTest=testsOfTypeOrdered[finalTestname[0][0]]
          finalTestFilePath=finalTest[0][0]
 

        #choose category if not defined (arithemtic, recall etc)
        #retrieve past performance from ontology for specified category
      tmpList=finalTestname.split('#')
      res.test=tmpList[1]

      rospack = rospkg.RosPack()
      localPackagePath=rospack.get_path('rapp_cognitive_exercise')
      #res.trace.append(localPackagePath)
      #tmp=userPerformanceResponse.tests[0]

      currentTimestamp = str(int(time.time()))
      edibleTime=datetime.now().strftime('%d/%m/%Y %H:%M:%S')
      #res.trace.append(edibleTime)
     
      
      
      finalTestFilePath=localPackagePath+finalTestFilePath
      res.trace.append(finalTestFilePath)
      tree = ET.parse(finalTestFilePath)
      root = tree.getroot()
      
      for question in root.find('Questions'):        
        res.questions.append(question.find("body").text)
        res.correctAnswers.append(question.find("correctAnswer").text)
        line=StringArrayMsg()
        for answers in question.findall('answer'):          
          line.s.append(answers.find("body").text)        
        res.answers.append(line)
      
      res.success=True  
       
    except IndexError:
      res.trace.append("Null pointer exception.. some argument was empty")
      res.error="Null pointer exception.. some argument was empty"
      res.success=False
      #print "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
    except IOError:      
      res.success=False
      res.trace.append("IO Error, cant open file or read data")
      res.error="IO Error, cant open file or read data"
    return res

  def organizeUserPerformance(self,userPerf):
    d=OrderedDict()
    for i in range(len(userPerf.tests)):
      tlist=[userPerf.tests[i],userPerf.scores[i],userPerf.difficulty[i],userPerf.variation[i]]
      d[int(userPerf.timestamps[i])]=[tlist]
      #testTypesDict[s]=['1']
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
    
  def organizeTestsOfType(self,testsOfType):
    d=dict()
    for i in range(len(testsOfType.tests)):
      tlist=[testsOfType.file_paths[i],testsOfType.difficulty[i],testsOfType.variation[i],testsOfType.subtype[i]]
      d[testsOfType.tests[i]]=[tlist]
    return d   

    
    
