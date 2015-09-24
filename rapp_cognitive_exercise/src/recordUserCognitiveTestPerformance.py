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
import calendar
import time
from datetime import datetime
from os.path import expanduser
from collections import OrderedDict



from rapp_platform_ros_communications.srv import (
  createOntologyAliasSrv,
  createOntologyAliasSrvRequest,
  createOntologyAliasSrvResponse,
  recordUserCognitiveTestPerformanceSrv,
  recordUserCognitiveTestPerformanceSrvRequest,
  recordUserCognitiveTestPerformanceSrvResponse,
  recordUserPerformanceCognitiveTestsSrv,
  recordUserPerformanceCognitiveTestsSrvRequest,
  recordUserPerformanceCognitiveTestsSrvResponse
  )
  
from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  ) 

class RecordUserCognitiveTestPerformance:

  def recordPerformance(self,req):
    try:
      res = recordUserCognitiveTestPerformanceSrvResponse() 
      
       
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
        
            
      
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_record_user_cognitive_tests_performance')
      if(not serv_topic):
        rospy.logerror("rapp_knowrob_wrapper_record_user_cognitive_tests_performance not found")
        res.trace.append("rapp_knowrob_wrapper_record_user_cognitive_tests_performance topic param not found")
        res.error="rapp_knowrob_wrapper_record_user_cognitive_tests_performance topic param not found"
        res.success=False
        return res
      rospy.wait_for_service(serv_topic)
      knowrob_service = rospy.ServiceProxy(serv_topic, recordUserPerformanceCognitiveTestsSrv)
      userPerformanceEntry = recordUserPerformanceCognitiveTestsSrvRequest()
      userPerformanceEntry.test=req.test 
      userPerformanceEntry.test_type=req.testType
      userPerformanceEntry.patient_ontology_alias=createOntologyAliasResponse.ontology_alias
      userPerformanceEntry.timestamp=int(time.time())
      userPerformanceEntry.score=req.score
      userPerformanceEntryResponse = knowrob_service(userPerformanceEntry)
      if(userPerformanceEntryResponse.success!=True):
        res.trace=userPerformanceEntryResponse.trace
        res.trace.append("Submitting query to ontology failed, either test or user ontology alias do not exist or test not of the given type")
        res.error=userPerformanceEntryResponse.error+"Submitting query to ontology failed, either test or user ontology alias do not exist or test not of the given type"
        res.success=False
        return res
      else:
        res.success=True
        res.userCognitiveTestPerformanceEntry=userPerformanceEntryResponse.cognitive_test_performance_entry
        
    
    except IndexError:
      res.trace.append("Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format")
      res.success=False
      #print "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
    except IOError:
      print "Error: can\'t find login file or read data" 
      res.success=False
      res.trace.append("Error: can\'t find login file or read data")
    return res

 

    
    
