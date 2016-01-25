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
from helper_functions import CognitiveExerciseHelperFunctions



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

## @class RecordUserCognitiveTestPerformance
# @brief Provides the necessary functions for recording the performance of a cognitive test of a user in the ontology
#
# It implements the cognitive exercise record user cognitive test performance service
class RecordUserCognitiveTestPerformance:

  ## @brief The callback function of the cognitive exercise record user cognitive test performance service
  # @param req [rapp_platform_ros_communications::recordUserCognitiveTestPerformanceSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::recordUserCognitiveTestPerformanceSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception AIOError
  def recordPerformance(self,req):
    try:
      res = recordUserCognitiveTestPerformanceSrvResponse()
      
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')
      knowrob_service = rospy.ServiceProxy(serv_topic, createOntologyAliasSrv)
      createOntologyAliasReq = createOntologyAliasSrvRequest()
      createOntologyAliasReq.username=req.username
      createOntologyAliasResponse = knowrob_service(createOntologyAliasReq)
      if(createOntologyAliasResponse.success!=True):
        res.trace=createOntologyAliasResponse.trace
        res.error=createOntologyAliasResponse.error
        res.success=False
        return res

      serv_topic = rospy.get_param('rapp_knowrob_wrapper_record_user_cognitive_tests_performance')
      knowrob_service = rospy.ServiceProxy(serv_topic, recordUserPerformanceCognitiveTestsSrv)
      userPerformanceEntry = recordUserPerformanceCognitiveTestsSrvRequest()
      userPerformanceEntry.test=req.test
      #userPerformanceEntry.patient_ontology_alias=createOntologyAliasResponse.ontology_alias
      userPerformanceEntry.patient_ontology_alias=CognitiveExerciseHelperFunctions.getUserOntologyAlias(req.username)
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
    except IOError:
      print "Error: can\'t find login file or read data"
      res.success=False
      res.trace.append("Error: can\'t find login file or read data")
    return res





