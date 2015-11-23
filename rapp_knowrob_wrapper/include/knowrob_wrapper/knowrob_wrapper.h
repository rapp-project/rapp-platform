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

class KnowrobWrapper
{
  private:
    ros::NodeHandle nh_;
    json_prolog::Prolog pl;
    ros::ServiceClient mysql_write_client;
    ros::ServiceClient mysql_fetch_client;
    ros::ServiceClient mysql_update_client;

  public:

    KnowrobWrapper(ros::NodeHandle nh);
    std::string get_ontology_alias(std::string user_id);
    std::string create_ontology_alias_for_new_user(std::string user_id);


    rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response  subclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req);
    rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response  superclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req);

    rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response  isSubSuperclassOfQuery(rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request req);
        //std::vector<std::string> createInstanceQuery(std::string caller_arguments);
    rapp_platform_ros_communications::createInstanceSrv::Response createInstanceQuery(rapp_platform_ros_communications::createInstanceSrv::Request req);
    rapp_platform_ros_communications::ontologyLoadDumpSrv::Response dumpOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req);
    rapp_platform_ros_communications::ontologyLoadDumpSrv::Response loadOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req);
    rapp_platform_ros_communications::assertRetractAttributeSrv::Response assertAttributeValue(rapp_platform_ros_communications::assertRetractAttributeSrv::Request req);

    rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response user_instances_of_class(rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request req);

    rapp_platform_ros_communications::createOntologyAliasSrv::Response create_ontology_alias(rapp_platform_ros_communications::createOntologyAliasSrv::Request req);

    rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response user_performance_cognitve_tests(rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request req);

    rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response create_cognitve_tests(rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request req);

    rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response cognitive_tests_of_type(rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request req);

    rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response record_user_cognitive_tests_performance(rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request req);

    rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response clear_user_cognitive_tests_performance_records(rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request req);


    //std::vector<std::string> userInstancesFromClassQuery(std::string ontology_class);
    //std::vector<std::string> checkIfClassExists(std::string classValue);
    //
};
