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
  userScoreHistoryForAllCategoriesSrvResponse,
  fetchDataSrv,
  fetchDataSrvRequest,
  fetchDataSrvResponse
  )

from rapp_platform_ros_communications.msg import (  
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

  @staticmethod
  ## @brief Queries the ontology and returns the cognitive test types available
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return returnWithError [bool] True if a non recoverable error occured, and the service must immediately return with an error report
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return testTypesList [list] The list of the available tests as they were read from the ontology
  def getTestTypesFromOntology():
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_subclasses_of_topic')
    knowrob_service = rospy.ServiceProxy(serv_topic, ontologySubSuperClassesOfSrv)
    testTypesReq = ontologySubSuperClassesOfSrvRequest()
    testTypesReq.ontology_class="CognitiveTests"
    testTypesResponse = knowrob_service(testTypesReq)
    if(testTypesResponse.success!=True):     
      testTypesResponse.trace.append("cannot load test categories from ontology")
      raise AppError(testTypesResponse.error+"cannot load test categories from ontology",testTypesResponse.trace)      
    testTypesList=[]
    for s in testTypesResponse.results:
      tmpList=s.split('#')
      testTypesList.append(tmpList[1])
    return testTypesList

  @staticmethod
  ## @brief Queries the MySQL database through the MySQL wrapper and returns the user's language
  # @param username [string] The username of the user as is in the MySQL database
  # @param res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  #
  # @return res [rapp_platform_ros_communications::testSelectorSrvResponse::Response&] The output arguments of the service as defined in the testSelectorSrv
  # @return userLanguage [string] The user's language setting
  def getUserLanguage(username,res):
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_fetch_data_topic')	
    knowrob_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    fetchDataSrvReq = fetchDataSrvRequest()
    fetchDataSrvReq.req_cols=["language"]
    fetchDataSrvReq.where_data=[StringArrayMsg(s=["username",username])]
    fetchDataSrvResponse = knowrob_service(fetchDataSrvReq)
    if(fetchDataSrvResponse.success.data!=True): 
      raise AppError(fetchDataSrvResponse.trace[0], fetchDataSrvResponse.trace)      
    res.language=fetchDataSrvResponse.res_data[0].s[0]
    return fetchDataSrvResponse.res_data[0].s[0]

  @staticmethod
  def determineTestTypeListForReturningScoresOrHistory(testType,testTypesList):
    if(not testType==""):
      if(testType not in testTypesList):
        error="invalid test type, not contained in ontology subclasses of cognitive test types"
        raise AppError(error,error)        
      testTypesList=[]
      testTypesList.append(req.testType)    
    return testTypesList
  
  @staticmethod  
  ## @brief Organizes the user's performance entries by timestamp
  # @param d [dict] The dictionary containing the user's performance entries
  #
  # @return d [OrderedDict] The dictionary containing the user's performance entries organized by timestamp
  def organizeUserPerformanceByTimestamp(userPerf):
    d=OrderedDict()
    for i in range(len(userPerf.tests)):
      tlist=[userPerf.tests[i],userPerf.scores[i],userPerf.difficulty[i], userPerf.subtypes[i]]
      d[int(userPerf.timestamps[i])]=[tlist]      
    d=OrderedDict(sorted(d.items(), key=lambda t: t[0], reverse=True)) #Order is descending    
    return d
