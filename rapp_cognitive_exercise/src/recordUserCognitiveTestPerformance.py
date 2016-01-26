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
import time
from helper_functions import CognitiveExerciseHelperFunctions
from app_error_exception import AppError
from rapp_platform_ros_communications.srv import ( 
  recordUserCognitiveTestPerformanceSrvResponse,
  recordUserPerformanceCognitiveTestsSrv,
  recordUserPerformanceCognitiveTestsSrvRequest
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
      userOntologyAlias=CognitiveExerciseHelperFunctions.getUserOntologyAlias(req.username)      
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_record_user_cognitive_tests_performance')
      knowrob_service = rospy.ServiceProxy(serv_topic, recordUserPerformanceCognitiveTestsSrv)
      userPerformanceEntry = recordUserPerformanceCognitiveTestsSrvRequest()
      userPerformanceEntry.test=req.test    
      userPerformanceEntry.patient_ontology_alias=userOntologyAlias
      userPerformanceEntry.timestamp=int(time.time())
      userPerformanceEntry.score=req.score
      userPerformanceEntryResponse = knowrob_service(userPerformanceEntry)      
      if(userPerformanceEntryResponse.success!=True):
        error=userPerformanceEntryResponse.error+"Submitting query to ontology failed, either test or user ontology alias do not exist or test not of the given type"        
        raise AppError(error,error)
      else:
        res.success=True
        res.userCognitiveTestPerformanceEntry=userPerformanceEntryResponse.cognitive_test_performance_entry
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
