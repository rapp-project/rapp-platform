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
from collections import OrderedDict
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
  returnTestsOfTypeSubtypeDifficultySrv,
  returnTestsOfTypeSubtypeDifficultySrvResponse,
  returnTestsOfTypeSubtypeDifficultySrvRequest

  )
from rapp_platform_ros_communications.msg import (
  CognitiveExercisesMsg
  )

## @class TestSelector
# @brief Provides the necessary functions for selecting a cognitive exercise test
#
# It implements the cognitive exercise chooser service
class ReturnTests:

  ## @brief The callback function of the cognitive exercise chooser service, all other functions of the class are called from within this function
  ## @brief The cognitive exercise chooser service callback
  # @param req [rapp_platform_ros_communications::testSelectorSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception AIOError
  # @exception Exception KeyError
  # @exception Exception AppError
  def returnTestsFunction(self,req):
    try:      
      res = returnTestsOfTypeSubtypeDifficultySrvResponse()
      
      testTypesList=CognitiveExerciseHelperFunctions.getTestTypesFromOntology()
      res.error="kati" 
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
