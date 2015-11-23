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
//#include <rapp_platform_ros_communications/OntologySimpleQuerySrv.h>
//#include <rapp_platform_ros_communications/DbWrapperSrv.h>
//#include <rapp_platform_ros_communications/StringArrayMsg.h>
#include <ros/ros.h>

class KnowrobWrapperCommunications
{
  private:
    ros::NodeHandle nh_;
    KnowrobWrapper knowrob_wrapper;


    ros::ServiceServer subclasses_of_service_;
    std::string subclasses_of_service_topic_;

    ros::ServiceServer superclasses_of_service_;
    std::string superclasses_of_service_topic_;

    ros::ServiceServer is_subsuperclass_of_service_;
    std::string is_subsuperclass_of_service_topic_;

    ros::ServiceServer createInstanceService_;
    std::string createInstanceServiceTopic_;

    ros::ServiceServer dumpOntologyService_;
    std::string dumpOntologyServiceTopic_;

    ros::ServiceServer loadOntologyService_;
    std::string loadOntologyServiceTopic_;

    ros::ServiceServer user_instances_of_class_service_;
    std::string user_instances_of_class_topic_;

    ros::ServiceServer create_ontology_alias_service_;
    std::string create_ontology_alias_topic_;

    ros::ServiceServer user_performance_cognitve_tests_service_;
    std::string user_performance_cognitve_tests_topic_;

    ros::ServiceServer create_cognitve_tests_service_;
    std::string create_cognitve_tests_topic_;

    ros::ServiceServer cognitive_tests_of_type_service_;
    std::string cognitive_tests_of_type_topic_;

    ros::ServiceServer record_user_cognitive_tests_performance_service_;
    std::string record_user_cognitive_tests_performance_topic_;

    ros::ServiceServer clear_user_cognitive_tests_performance_records_service_;
    std::string clear_user_cognitive_tests_performance_records_topic_;



    //ros::ServiceServer userInstancesFromClassService_;
    //std::string userInstancesFromClassServiceTopic_;

    //ros::ServiceServer assignAttributeValueService_;
    //std::string assignAttributeValueServiceTopic_;
    //ros::ServiceServer instanceFromClassService_;
    //std::string instanceFromClassServiceTopic_;



  public:

    KnowrobWrapperCommunications();

    bool subclassesOfCallback(
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res);

    bool superclassesOfCallback(
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
      rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res);


    bool isSubSuperclassOfCallback(
      rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request& req,
      rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response& res);

    bool createInstanceCallback(
      rapp_platform_ros_communications::createInstanceSrv::Request& req,
      rapp_platform_ros_communications::createInstanceSrv::Response& res);

    bool dumpOntologyCallback(
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res);

    bool loadOntologyCallback(
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
      rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res);

    bool user_instances_of_class_callback(
      rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request& req,
      rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response& res);

    bool create_ontology_alias_callback(
      rapp_platform_ros_communications::createOntologyAliasSrv::Request& req,
      rapp_platform_ros_communications::createOntologyAliasSrv::Response& res);

    bool user_performance_cognitve_tests_callback(
      rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request& req,
      rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response& res);

    bool create_cognitve_tests_callback(
      rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request& req,
      rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response& res);

    bool cognitive_tests_of_type_callback(
      rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request& req,
      rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response& res);

    bool record_user_cognitive_tests_performance_callback(
      rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request& req,
      rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response& res);

    bool clear_user_cognitive_tests_performance_records_callback(
      rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request& req,
      rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response& res);



    //bool userInstancesFromClassCallback(
      //rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      //rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);

    //bool assignAttributeValueCallback(
      //rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      //rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);


};
