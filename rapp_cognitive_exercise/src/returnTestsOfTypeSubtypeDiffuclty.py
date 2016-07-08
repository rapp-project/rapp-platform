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

## @class ReturnTests
# @brief Provides the necessary functions for returning tests of given type
#
# It implements the cognitive exercise return tests of type service
class ReturnTests:

  ## @brief The callback function of the return tests of type cognitive exercise service
  # @param req [rapp_platform_ros_communications::returnTestsOfTypeSubtypeDifficultySrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::returnTestsOfTypeSubtypeDifficultySrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception AIOError
  # @exception Exception KeyError
  # @exception Exception AppError
  def returnTestsFunction(self,req):
    try:      
      res = returnTestsOfTypeSubtypeDifficultySrvResponse()
      if(req.testType==""):
        error="Error, empty testType field"
        raise AppError(error, error)
      supportedLanguages=CognitiveExerciseHelperFunctions.getTestLanguagesFromOntology()      
      keepEntries = dict()
      testCount=0      
      for currentLanguage in supportedLanguages:
        cognitiveTestsOfTypeResponse=CognitiveExerciseHelperFunctions.getCognitiveTestsOfType(req.testType,currentLanguage)        
        tests=CognitiveExerciseHelperFunctions.filterTestsbyDifficultyAndSubtype(cognitiveTestsOfTypeResponse,req.difficulty,req.testSubType)       
        if(req.language=="" or currentLanguage==req.language):
          for k, v in tests.items():
            if(not k in keepEntries):
              currentCognitiveTest =  CognitiveExercisesMsg()
              currentCognitiveTest.testName=k.split('#')[1]
              currentCognitiveTest.testType=req.testType
              currentCognitiveTest.testSubType=(v[0][2]).split('#')[1]
              currentCognitiveTest.testSubType=(v[0][2]).split('#')[1]
              currentCognitiveTest.difficulty=(v[0][1])
              currentCognitiveTest.test_id=(v[0][3])
              currentCognitiveTest.languages.append(currentLanguage)              
              res.cognitiveExercises.append(currentCognitiveTest)
              keepEntries[k]=testCount
              testCount=testCount+1
            else:
              res.cognitiveExercises[keepEntries[k]].languages.append(currentLanguage)
      res.totalNumberOfTestsReturned=testCount
      res.success=True
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.error="IndexError: "+str(e)
      res.success=False
      CognitiveExerciseHelperFunctions.traceError(res.error,res.trace)
    except IOError, e:
      res.success=False
      res.trace.append("IOError: "+str(e))
      res.error="IOError: "+str(e)
      CognitiveExerciseHelperFunctions.traceError(res.error,res.trace)
    except KeyError, e:
      res.success=False
      res.trace.append('"KeyError (probably invalid cfg/.yaml parameter) "%s"' % str(e))
      res.error='"KeyError (probably invalid cfg/.yaml parameter) "%s"' % str(e)
      CognitiveExerciseHelperFunctions.traceError(res.error,res.trace)
    except AppError as e:
      AppError.passErrorToRosSrv(e,res)
      res.totalNumberOfTestsReturned=0
    return res
