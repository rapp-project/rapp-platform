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

#include <knowrob_wrapper/knowrob_wrapper.h>
#include <ros/ros.h>

/** 
* @class KnowrobWrapperCommunications 
* @brief Class KnowrobWrapperCommunications uptakes the task of handling the ROS service callbacks 
*/ 
class KnowrobWrapperCommunications
{
  private:
    /**< The ROS node handle */
    ros::NodeHandle nh_;
    
    /**< Object of type knowrob_wrapper */
    KnowrobWrapper knowrob_wrapper;

    /**< The subclasses_of service server */
    ros::ServiceServer subclasses_of_service_;
    
    /**< Member variable holding the subclasses_of ROS service topic */
    std::string subclasses_of_service_topic_;

	/**< The superclasses_of service server */
    ros::ServiceServer superclasses_of_service_;
    /**< Member variable holding the superclasses_of ROS service topic */
    std::string superclasses_of_service_topic_;

	/**< The is_subsuperclass service server */
    ros::ServiceServer is_subsuperclass_of_service_;
    /**< Member variable holding the is_subsuperclass ROS service topic */
    std::string is_subsuperclass_of_service_topic_;

	/**< The createInstanceService service server */
    ros::ServiceServer createInstanceService_;
    /**< Member variable holding the createInstanceServiceTopic ROS service topic */
    std::string createInstanceServiceTopic_;

	/**< The dumpOntologyService service server */
    ros::ServiceServer dumpOntologyService_;
    /**< Member variable holding the dumpOntologyServiceTopic ROS service topic */
    std::string dumpOntologyServiceTopic_;

	/**< The loadOntologyService service server */
    ros::ServiceServer loadOntologyService_;
    /**< Member variable holding the loadOntologyServiceTopic ROS service topic */
    std::string loadOntologyServiceTopic_;

	/**< The user_instances_of_class service server */
    ros::ServiceServer user_instances_of_class_service_;
    /**< Member variable holding the user_instances_of_class ROS service topic */
    std::string user_instances_of_class_topic_;
    
	/**< The create_ontology_alias service server */
    ros::ServiceServer create_ontology_alias_service_;
    /**< Member variable holding the create_ontology_alias ROS service topic */
    std::string create_ontology_alias_topic_;

	/**< The user_performance_cognitve_tests service server */
    ros::ServiceServer user_performance_cognitve_tests_service_;
    /**< Member variable holding the user_performance_cognitve_tests ROS service topic */
    std::string user_performance_cognitve_tests_topic_;

	/**< The create_cognitve_tests service server */
    ros::ServiceServer create_cognitve_tests_service_;
    /**< Member variable holding the create_cognitve_tests ROS service topic */
    std::string create_cognitve_tests_topic_;

	/**< The cognitive_tests_of_type service server */
    ros::ServiceServer cognitive_tests_of_type_service_;
    /**< Member variable holding the cognitive_tests_of_type ROS service topic */
    std::string cognitive_tests_of_type_topic_;

	/**< The record_user_cognitive_tests_performance service server */
    ros::ServiceServer record_user_cognitive_tests_performance_service_;
    /**< Member variable holding the record_user_cognitive_tests_performance ROS service topic */
    std::string record_user_cognitive_tests_performance_topic_;

	/**< The clear_user_cognitive_tests_performance_records service server */
    ros::ServiceServer clear_user_cognitive_tests_performance_records_service_;
    /**< Member variable holding the clear_user_cognitive_tests_performance_records ROS service topic */
    std::string clear_user_cognitive_tests_performance_records_topic_;
    
	/**< The retract_user_ontology_alias service server */
    ros::ServiceServer retract_user_ontology_alias_service_;
    /**< Member variable holding the retract_user_ontology_alias ROS service topic */
    std::string retract_user_ontology_alias_topic_;    
    

    //ros::ServiceServer userInstancesFromClassService_;
    //std::string userInstancesFromClassServiceTopic_;

    //ros::ServiceServer assignAttributeValueService_;
    //std::string assignAttributeValueServiceTopic_;
    //ros::ServiceServer instanceFromClassService_;
    //std::string instanceFromClassServiceTopic_;

  public:

	/**  
	*  @brief Default constructor 
	*/ 
    KnowrobWrapperCommunications();
    
	/** 
	* @brief Serves the subclassesOf ROS service callback 
	* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool subclassesOfCallback(
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res);
      
	/** 
	* @brief Serves the superlassesOf ROS service callback 
	* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool superclassesOfCallback(
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res);

	/** 
	* @brief Serves the isSubSuperclassOf ROS service callback 
	* @param req [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool isSubSuperclassOfCallback(
      rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request& req,
      rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response& res);
      
	/** 
	* @brief Serves the createInstance ROS service callback 
	* @param req [rapp_platform_ros_communications::createInstanceSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::createInstanceSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool createInstanceCallback(
      rapp_platform_ros_communications::createInstanceSrv::Request& req,
      rapp_platform_ros_communications::createInstanceSrv::Response& res);

	/** 
	* @brief Serves the dumpOntology ROS service callback 
	* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool dumpOntologyCallback(
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res);

	/** 
	* @brief Serves the loadOntology ROS service callback 
	* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool loadOntologyCallback(
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res);

	/** 
	* @brief Serves the create_ontology_alias ROS service callback 
	* @param req [rapp_platform_ros_communications::createOntologyAliasSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::createOntologyAliasSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool create_ontology_alias_callback(
      rapp_platform_ros_communications::createOntologyAliasSrv::Request& req,
      rapp_platform_ros_communications::createOntologyAliasSrv::Response& res);

	/** 
	* @brief Serves the user_instances_of_class ROS service callback 
	* @param req [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool user_instances_of_class_callback(
      rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request& req,
      rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response& res);

	/** 
	* @brief Serves the user_performance_cognitve_tests ROS service callback 
	* @param req [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool user_performance_cognitve_tests_callback(
      rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request& req,
      rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response& res);

	/** 
	* @brief Serves the create_cognitve_tests ROS service callback 
	* @param req [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool create_cognitve_tests_callback(
      rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request& req,
      rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response& res);

	/** 
	* @brief Serves the cognitive_tests_of_type ROS service callback 
	* @param req [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool cognitive_tests_of_type_callback(
      rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request& req,
      rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response& res);

	/** 
	* @brief Serves the record_user_cognitive_tests_performance ROS service callback 
	* @param req [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool record_user_cognitive_tests_performance_callback(
      rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request& req,
      rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response& res);

	/** 
	* @brief Serves the clear_user_cognitive_tests_performance_records ROS service callback 
	* @param req [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool clear_user_cognitive_tests_performance_records_callback(
      rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request& req,
      rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response& res);

	/** 
	* @brief Serves the retract_user_ontology_alias ROS service callback 
	* @param req [rapp_platform_ros_communications::retractUserOntologyAliasSrv::Request&] The ROS service request 
	* @param res [rapp_platform_ros_communications::retractUserOntologyAliasSrv::Response&] The ROS service response 
	* @return bool - The success status of the call 
	*/ 
    bool retract_user_ontology_alias_callback(
      rapp_platform_ros_communications::retractUserOntologyAliasSrv::Request& req,
      rapp_platform_ros_communications::retractUserOntologyAliasSrv::Response& res);
     
};
