import rospy
import sys
import calendar
import time
from datetime import datetime
from os.path import expanduser
from collections import OrderedDict
from app_error_exception import AppError

from rapp_platform_ros_communications.srv import (
  ontologySubSuperClassesOfSrv,
  ontologySubSuperClassesOfSrvRequest,
  ontologySubSuperClassesOfSrvResponse,
  createOntologyAliasSrv,
  createOntologyAliasSrvRequest,
  createOntologyAliasSrvResponse,
  userPerformanceCognitveTestsSrv,
  userPerformanceCognitveTestsSrvRequest,
  userScoreHistoryForAllCategoriesSrv,
  userScoreHistoryForAllCategoriesSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  CognitiveExercisePerformanceRecordsMsg,
  ArrayCognitiveExercisePerformanceRecordsMsg,
  StringArrayMsg
  )


class CognitiveExerciseHelperFunctions:
  
  @staticmethod
  def getUserPerformanceRecordsForTestType(testType,userOntologyAlias):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_user_performance_cognitve_tests')
    userPerformanceReq=userPerformanceCognitveTestsSrvRequest()
    userPerformanceReq.test_type=testType
    userPerformanceReq.ontology_alias=userOntologyAlias
    knowrob_service = rospy.ServiceProxy(serv_topic, userPerformanceCognitveTestsSrv)
    return knowrob_service(userPerformanceReq)
  
  @staticmethod
    ## @brief Gets the users ontology alias and if it doesnt exist it creates it  
  # @param username [string] The user's username
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return ontologyAlias [string] The user's ontology alias
  def getUserOntologyAlias(username):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')      
    knowrob_service = rospy.ServiceProxy(serv_topic, createOntologyAliasSrv)
    createOntologyAliasReq = createOntologyAliasSrvRequest()
    createOntologyAliasReq.username=username
    createOntologyAliasResponse = knowrob_service(createOntologyAliasReq)
    if(createOntologyAliasResponse.success!=True):
      raise AppError(createOntologyAliasResponse.error, createOntologyAliasResponse.trace)      
    return createOntologyAliasResponse.ontology_alias
