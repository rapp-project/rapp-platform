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

#include <knowrob_wrapper/knowrob_wrapper_communications.h>
#include <ros/package.h>

/**  
* @brief Default constructor 
* Waits for services it depends on and declares the knowrob wrapper services callbacks
*/ 
KnowrobWrapperCommunications::KnowrobWrapperCommunications():knowrob_wrapper(nh_)
{
  ros::service::waitForService("json_prolog/query", -1);
  ros::service::waitForService("rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_write_data", -1);
  ros::service::waitForService("rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_fetch_data", -1); 

  if(!nh_.getParam("/rapp_knowrob_wrapper_subclasses_of_topic", subclasses_of_service_topic_))
  {
    ROS_ERROR("rapp_knowrob_wrapper_subclass_of_topic not found");
  }
  subclasses_of_service_ = nh_.advertiseService(subclasses_of_service_topic_,
    &KnowrobWrapperCommunications::subclassesOfCallback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_superclasses_of_topic", superclasses_of_service_topic_))
  {
    ROS_ERROR("rapp_knowrob_wrapper_superclass_of_topic not found");
  }
  superclasses_of_service_ = nh_.advertiseService(superclasses_of_service_topic_,
    &KnowrobWrapperCommunications::superclassesOfCallback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_is_subsuperclass_of_topic", is_subsuperclass_of_service_topic_))
  {
    ROS_ERROR("rapp_knowrob_wrapper_issubsuperclass_of_topic not found");
  }
  is_subsuperclass_of_service_ = nh_.advertiseService(is_subsuperclass_of_service_topic_,
    &KnowrobWrapperCommunications::isSubSuperclassOfCallback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_create_instance_topic", createInstanceServiceTopic_))
  {
    ROS_ERROR("rapp_knowrob_wrapper_create_instance_topic not found");
  }
  createInstanceService_ = nh_.advertiseService(createInstanceServiceTopic_,
    &KnowrobWrapperCommunications::createInstanceCallback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_dump_ontology_topic", dumpOntologyServiceTopic_))
  {
    ROS_ERROR("rapp_knowrob_wrapper_dump_ontology_topic not found");
  }
  dumpOntologyService_ = nh_.advertiseService(dumpOntologyServiceTopic_,
    &KnowrobWrapperCommunications::dumpOntologyCallback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_load_ontology_topic", loadOntologyServiceTopic_))
  {
    ROS_ERROR("ontology_load_ontology_topic not found");
  }
  loadOntologyService_ = nh_.advertiseService(loadOntologyServiceTopic_,
    &KnowrobWrapperCommunications::loadOntologyCallback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_user_instances_of_class", user_instances_of_class_topic_))
  {
    ROS_ERROR("ontologyuser_instances_of_class_topic not found");
  }
  user_instances_of_class_service_ = nh_.advertiseService(user_instances_of_class_topic_,
    &KnowrobWrapperCommunications::user_instances_of_class_callback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_create_ontology_alias", create_ontology_alias_topic_))
  {
    ROS_ERROR("rapp_knowrob_wrapper_create_ontology_alias_topic not found");
  }
  create_ontology_alias_service_ = nh_.advertiseService(create_ontology_alias_topic_,
    &KnowrobWrapperCommunications::create_ontology_alias_callback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_user_performance_cognitve_tests", user_performance_cognitve_tests_topic_))
  {
    ROS_ERROR("create_user_performance_cognitve_tests_topic not found");
  }
  user_performance_cognitve_tests_service_ = nh_.advertiseService(user_performance_cognitve_tests_topic_,
    &KnowrobWrapperCommunications::user_performance_cognitve_tests_callback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_create_cognitve_tests", create_cognitve_tests_topic_))
  {
    ROS_ERROR("create_cognitve_tests_topic not found");
  }
  create_cognitve_tests_service_ = nh_.advertiseService(create_cognitve_tests_topic_,
    &KnowrobWrapperCommunications::create_cognitve_tests_callback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_cognitive_tests_of_type", cognitive_tests_of_type_topic_))
  {
    ROS_ERROR("cognitive_tests_of_type not found");
  }
  cognitive_tests_of_type_service_ = nh_.advertiseService(cognitive_tests_of_type_topic_,
    &KnowrobWrapperCommunications::cognitive_tests_of_type_callback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_record_user_cognitive_tests_performance", record_user_cognitive_tests_performance_topic_))
  {
    ROS_ERROR("record_user_cognitive_tests_performance not found");
  }
  record_user_cognitive_tests_performance_service_ = nh_.advertiseService(record_user_cognitive_tests_performance_topic_,
    &KnowrobWrapperCommunications::record_user_cognitive_tests_performance_callback, this);

  if(!nh_.getParam("/rapp_knowrob_wrapper_clear_user_cognitive_tests_performance_records", clear_user_cognitive_tests_performance_records_topic_))
  {
    ROS_ERROR("rapp_knowrob_wrapper_clear_user_cognitive_tests_performance_records not found");
  }
  clear_user_cognitive_tests_performance_records_service_ = nh_.advertiseService(clear_user_cognitive_tests_performance_records_topic_,
    &KnowrobWrapperCommunications::clear_user_cognitive_tests_performance_records_callback, this);

  //if(!nh_.getParam("/ontology_user_instances_from_class_topic", userInstancesFromClassServiceTopic_))
  //{
    //ROS_ERROR("ontology_user_instances_from_class_topic");
  //}

  //userInstancesFromClassService_ = nh_.advertiseService(userInstancesFromClassServiceTopic_,
    //&KnowrobWrapperCommunications::userInstancesFromClassCallback, this);

  //if(!nh_.getParam("/ontology_assign_attribute_value", assignAttributeValueServiceTopic_))
  //{
    //ROS_ERROR("ontology_assign_attribute_value");
  //}

  //assignAttributeValueService_ = nh_.advertiseService(assignAttributeValueServiceTopic_,
    //&KnowrobWrapperCommunications::assignAttributeValueCallback, this);

  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req;
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;

  req.file_url=std::string("currentOntologyVersion.owl");
    res=knowrob_wrapper.loadOntologyQuery(req);
    if(res.success!=true)
    {
      ROS_ERROR("Ontology backup was not loaded.. Continuing with empty ontology");
      ROS_ERROR_STREAM(res.error);
    }
    else
    {
      ROS_INFO("Ontology backup successfully loaded");
    }
  ROS_INFO("KnowRob ROS wrapper initialized");  
}

/** 
* @brief Serves the subclassesOf ROS service callback 
* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::subclassesOfCallback(
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res)
{
  res=knowrob_wrapper.subclassesOfQuery(req);
  return true;
}

/** 
* @brief Serves the superlassesOf ROS service callback 
* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::superclassesOfCallback(
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res)
{
  res=knowrob_wrapper.superclassesOfQuery(req);
  return true;
}

/** 
* @brief Serves the isSubSuperclassOf ROS service callback 
* @param req [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::isSubSuperclassOfCallback(
  rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request& req,
  rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response& res)
{
  res=knowrob_wrapper.isSubSuperclassOfQuery(req);
  return true;
}

/** 
* @brief Serves the createInstance ROS service callback 
* @param req [rapp_platform_ros_communications::createInstanceSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::createInstanceSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::createInstanceCallback(
  rapp_platform_ros_communications::createInstanceSrv::Request& req,
  rapp_platform_ros_communications::createInstanceSrv::Response& res)
{
  res=knowrob_wrapper.createInstanceQuery(req);
  return true;
}

/** 
* @brief Serves the dumpOntology ROS service callback 
* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::dumpOntologyCallback(
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res)
{
  res=knowrob_wrapper.dumpOntologyQuery(req);
  return true;
}

/** 
* @brief Serves the loadOntology ROS service callback 
* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::loadOntologyCallback(
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res)
{
  res=knowrob_wrapper.loadOntologyQuery(req);
  return true;
}

/** 
* @brief Serves the create_ontology_alias ROS service callback 
* @param req [rapp_platform_ros_communications::createOntologyAliasSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::createOntologyAliasSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::create_ontology_alias_callback(
  rapp_platform_ros_communications::createOntologyAliasSrv::Request& req,
  rapp_platform_ros_communications::createOntologyAliasSrv::Response& res)
{
  res=knowrob_wrapper.create_ontology_alias(req);
  return true;
}

/** 
* @brief Serves the user_instances_of_class ROS service callback 
* @param req [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::user_instances_of_class_callback(
  rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request& req,
  rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response& res)
{
  res=knowrob_wrapper.user_instances_of_class(req);
  return true;
}

/** 
* @brief Serves the user_performance_cognitve_tests ROS service callback 
* @param req [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::user_performance_cognitve_tests_callback(
  rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request& req,
  rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response& res)
{
  res=knowrob_wrapper.user_performance_cognitve_tests(req);
  return true;
}

/** 
* @brief Serves the create_cognitve_tests ROS service callback 
* @param req [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::create_cognitve_tests_callback(
  rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request& req,
  rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response& res)
{
  res=knowrob_wrapper.create_cognitve_tests(req);
  return true;
}

/** 
* @brief Serves the cognitive_tests_of_type ROS service callback 
* @param req [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::cognitive_tests_of_type_callback(
  rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request& req,
  rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response& res)
{
  res=knowrob_wrapper.cognitive_tests_of_type(req);
  return true;
}

/** 
* @brief Serves the record_user_cognitive_tests_performance ROS service callback 
* @param req [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::record_user_cognitive_tests_performance_callback(
  rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request& req,
  rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response& res)
{
  res=knowrob_wrapper.record_user_cognitive_tests_performance(req);
  return true;
}

/** 
* @brief Serves the clear_user_cognitive_tests_performance_records ROS service callback 
* @param req [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request&] The ROS service request 
* @param res [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response&] The ROS service response 
* @return bool - The success status of the call 
*/ 
bool KnowrobWrapperCommunications::clear_user_cognitive_tests_performance_records_callback(
  rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request& req,
  rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response& res)
{
  res=knowrob_wrapper.clear_user_cognitive_tests_performance_records(req);
  return true;
}

//bool KnowrobWrapperCommunications::userInstancesFromClassCallback(
  //rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  //rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
//{
  ////std::cout<<req.return_cols[0].data;
  //std::vector<std::string> res_ =
    //knowrob_wrapper.userInstancesFromClassQuery(req.query_term.data);
  //for(unsigned int i = 0 ; i < res_.size() ; i++)
  //{
    //std_msgs::String s;
    //s.data = res_[i];
    //res.results.push_back(s);
  //}

  //return true;
//}

//bool KnowrobWrapperCommunications::assignAttributeValueCallback(
  //rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  //rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
//{
  ////std::cout<<req.return_cols[0].data;
  //std::vector<std::string> res_ =
    //knowrob_wrapper.assignAttributeValueQuery(req.query_term.data);
  //for(unsigned int i = 0 ; i < res_.size() ; i++)
  //{
    //std_msgs::String s;
    //s.data = res_[i];
    //res.results.push_back(s);
  //}

  //return true;
//}

