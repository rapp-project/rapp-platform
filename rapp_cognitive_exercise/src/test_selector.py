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

import rospy
import sys
import xml.etree.ElementTree as ET
import calendar
import time
from datetime import datetime
from os.path import expanduser

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
  userPerformanceCognitveTestsSrvResponse
  )
  
from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  ) 

class TestSelector:

  def chooserFunction(self,req):
    try:
      res = testSelectorSrvResponse()      
      
      #must insert code here to check if testType exists. use subclassesof function of knowrob wrapper, super class is CognitiveTests
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
        res.trace=testTypesResponse.trace
        res.trace.append("cannot load test categories from ontology")
        res.error=testTypesResponse.error+"cannot load test categories from ontology"
        res.success=False
        return res
      testTypesDict=dict()
      for s in testTypesResponse.results:
        #res.trace.append(s)
        testTypesDict[s]=['1']
      if ("http://knowrob.org/kb/knowrob.owl#"+req.testType not in testTypesDict): 
        #res.trace.append(testTypesDict
        res.trace.append("testType provided does not exist")
        res.error="testType provided does not exist"
        res.success=False
        return res
      
      #testTypes=testTypesResponse.results
      #if(req.testType!=""):
        #if any(req.testType in s for s in testTypes):
          #print "something"
          
      #check if username exists, if not prompt for create user in mysql DB
      #check if ontology alias exists for user, if not create it
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
        res.trace=createOntologyAliasResponse.trace
        res.error=createOntologyAliasResponse.error
        res.success=False
        return res
      
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests')
      if(not serv_topic):
        rospy.logerror("rapp_knowrob_wrapper_user_performance_cognitve_tests")
        res.trace.append("mysql_wrapper_rapp_read_data topic param not found")
        res.error="mysql_wrapper_rapp_read_data topic param not found"
        res.success=False
        return res
     
      if (req.testType==""):
        print "kati"
      else:        
        userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
        userPerformanceReq.test_type=req.testType
        userPerformanceReq.ontology_alias=createOntologyAliasResponse.ontology_alias
        knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)       
        userPerformanceResponse = knowrob_service(userPerformanceReq)
        if(userPerformanceResponse.success!=True):
          #means no performance records exist. so you choose random test, test1, variation1
          res.trace=userPerformanceResponse.trace
          res.error=userPerformanceResponse.error
          res.success=False
          return res
        
      #choose category if not defined (arithemtic, recall etc)
      #retrieve past performance from ontology for specified category
      
      #tmp=expanduser('~') 
        tmp=userPerformanceResponse.tests[0]
        res.trace.append(tmp)
        timestamp = str(int(time.time()))
        edibleTime=datetime.now().strftime('%d/%m/%Y %H:%M:%S')
        res.trace.append(edibleTime)
        res.trace.append(timestamp)     
        
        
        testFilePath=tmp+"/rapp_platform_catkin_ws/src/rapp-platform/rapp_cognitive_exercise/cognitiveTests/arithmeticTest1.xml"
        tree = ET.parse(testFilePath)
        root = tree.getroot()
        
        for question in root.find('Questions'):        
          res.questions.append(question.find("body").text)
          line=StringArrayMsg()
          for answers in question.findall('answer'):          
            line.s.append(answers.find("body").text)        
          res.answers.append(line)
            
    
      #check if ontology alias of user exists      
    except IndexError:
      res.trace.append("Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format")
      res.success=False
      #print "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
    except IOError:
      print "Error: can\'t find login file or read data" 
      res.success=False
      res.trace.append("Error: can\'t find login file or read data")
    return res
