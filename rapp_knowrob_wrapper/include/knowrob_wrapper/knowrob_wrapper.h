/******************************************************************************
Copyright 2015 RAPP

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

  Author: Athanassios Kintsakis
  contact: akintsakis@issel.ee.auth.gr

******************************************************************************/

#include <string>
#include <iostream>
#include <json_prolog/prolog.h>
#include <rapp_platform_ros_communications/fetchDataSrv.h>
#include <rapp_platform_ros_communications/writeDataSrv.h>
#include <rapp_platform_ros_communications/updateDataSrv.h>
#include <rapp_platform_ros_communications/StringArrayMsg.h>
#include <rapp_platform_ros_communications/createInstanceSrv.h>
#include <rapp_platform_ros_communications/ontologyInstancesOfSrv.h>
#include <rapp_platform_ros_communications/ontologyLoadDumpSrv.h>
#include <rapp_platform_ros_communications/ontologySubSuperClassesOfSrv.h>
#include <rapp_platform_ros_communications/assertRetractAttributeSrv.h>
#include <rapp_platform_ros_communications/ontologyIsSubSuperClassOfSrv.h>
#include <rapp_platform_ros_communications/returnUserInstancesOfClassSrv.h>
#include <rapp_platform_ros_communications/createOntologyAliasSrv.h>
#include <rapp_platform_ros_communications/userPerformanceCognitveTestsSrv.h>
#include <rapp_platform_ros_communications/createCognitiveExerciseTestSrv.h>
#include <rapp_platform_ros_communications/cognitiveTestsOfTypeSrv.h>
#include <rapp_platform_ros_communications/recordUserPerformanceCognitiveTestsSrv.h>
#include <rapp_platform_ros_communications/clearUserPerformanceCognitveTestsSrv.h>

/** 
* @class KnowrobWrapper 
* @brief Class KnowrobWrapperCommunications contains all the necessary knowrob wrapper functions 
*/ 
class KnowrobWrapper
{
  private:
	/**< The ROS node handle */
    ros::NodeHandle nh_;
    
    /**< The json prolog handle */
    json_prolog::Prolog pl;
    
    /**< The mysql write to tblUser client server */
    ros::ServiceClient mysql_write_client;
    
    /**< The mysql fetch from tblUser client server */
    ros::ServiceClient mysql_fetch_client;
    
    /**< The mysql update tblUser client server */
    ros::ServiceClient mysql_update_client;

  public:

	/**  
	* @brief Default constructor 
	*/ 
    KnowrobWrapper(ros::NodeHandle nh);
    
    void dump_ontology_now();
    
    //bool checkIfStringContainsString(std::string a, std::string b);
    //bool checkIfStringVectorContainsString(std::vector<std::string> vec, std::string a);
    
	/** 
	* @brief Returns the ontology alias of the user
	* @param user_id [string] The username of the user 
	* @return ontology_alias [string] The ontology_alias of the user or possible error
	*/
    std::string get_ontology_alias(std::string user_id);
    
    /** 
	* @brief Creates a new ontology alias for a user
	* @param user_id [string] The username of the user 
	* @return ontology_alias [string] The ontology_alias of the user or possible error
	*/
    std::string create_ontology_alias_for_new_user(std::string user_id);

	/** 
	* @brief Implements the subclassesOf ROS service 
	* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response  subclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req);
  
	/** 
	* @brief Implements the superclassesOf ROS service 
	* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
	*/   
    rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response  superclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req);

	/** 
	* @brief Implements the isSubSuperclass ROS service 
	* @param req [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response&] The ROS service response 
	*/   
    rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response  isSubSuperclassOfQuery(rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request req);

	/** 
	* @brief Implements the createInstance ROS service 
	* @param req [rapp_platform_ros_communications::createInstanceSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::createInstanceSrv::Response&] The ROS service response 
	*/  
    rapp_platform_ros_communications::createInstanceSrv::Response createInstanceQuery(rapp_platform_ros_communications::createInstanceSrv::Request req);

	/** 
	* @brief Implements the dumpOntology ROS service 
	* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::ontologyLoadDumpSrv::Response dumpOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req);

	/** 
	* @brief Implements the loadOntology ROS service 
	* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::ontologyLoadDumpSrv::Response loadOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req);

	/** 
	* @brief Implements the assertRetractAttribute ROS service 
	* @param req [rapp_platform_ros_communications::assertRetractAttributeSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::assertRetractAttributeSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::assertRetractAttributeSrv::Response assertAttributeValue(rapp_platform_ros_communications::assertRetractAttributeSrv::Request req);

	/** 
	* @brief Implements the returnUserInstancesOfClass ROS service 
	* @param req [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response user_instances_of_class(rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request req);

	/** 
	* @brief Implements the create_ontology_alias ROS service 
	* @param req [rapp_platform_ros_communications::createOntologyAliasSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::createOntologyAliasSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::createOntologyAliasSrv::Response create_ontology_alias(rapp_platform_ros_communications::createOntologyAliasSrv::Request req);

	/** 
	* @brief Implements the user_performance_cognitve_tests ROS service 
	* @param req [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response user_performance_cognitve_tests(rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request req);

	/** 
	* @brief Implements the create_cognitve_tests ROS service 
	* @param req [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response create_cognitve_tests(rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request req);

	/** 
	* @brief Implements the cognitive_tests_of_type ROS service 
	* @param req [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response cognitive_tests_of_type(rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request req);

	/** 
	* @brief Implements the record_user_cognitive_tests_performance ROS service 
	* @param req [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response record_user_cognitive_tests_performance(rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request req);

	/** 
	* @brief Implements the clear_user_cognitive_tests_performance_records ROS service 
	* @param req [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request&] The ROS service request 
	* @return res [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response&] The ROS service response 
	*/ 
    rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response clear_user_cognitive_tests_performance_records(rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request req);

    //std::vector<std::string> userInstancesFromClassQuery(std::string ontology_class);
    //std::vector<std::string> checkIfClassExists(std::string classValue);
    //
};
