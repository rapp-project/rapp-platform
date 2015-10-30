/**
MIT License (MIT)

Copyright (c) <2014> <Rapp Project EU>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Author: Athanassios Kintsakis
contact: akintsakis@issel.ee.auth.gr
**/
#include <knowrob_wrapper/knowrob_wrapper_communications.h>
#include <ros/package.h>
bool checkIfFileExists(const char* fname)
{
  if( access( fname, F_OK ) != -1 ) {
      return true;
  }
  return false;  
}

KnowrobWrapperCommunications::KnowrobWrapperCommunications():knowrob_wrapper(nh_)
{
  ros::service::waitForService("json_prolog/query", -1);
  ros::service::waitForService("rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_write_data", -1);
  ros::service::waitForService("rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_fetch_data", -1);
  //ros::service::waitForService("rapp_mysql_wrapper_users_ontology_instances_fetch_data_topic", -1);
  //ros::service::waitForService("rapp_mysql_wrapper_users_ontology_instances_write_data_topic", -1);
    
  //ros::service::waitForService("json_prolog/query", -1);  for DB service

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
  //rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;
  //std::string path = ros::package::getPath("rapp_knowrob_wrapper");



  req.file_url=std::string("currentOntologyVersion.owl");  
  //const char * c = req.file_url.c_str();  
  //if(!checkIfFileExists(c))
  //{
    //ROS_ERROR("Ontology backup was not loaded, backup file does not exist.. Continuing with empty ontology");  
  //}
  //else
  //{
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
 // }  
  
  
  //ROS_ERROR(path);
  
  
  ROS_INFO("KnowRob ROS wrapper initialized");
}

bool KnowrobWrapperCommunications::subclassesOfCallback(
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res)
{
  res=knowrob_wrapper.subclassesOfQuery(req);
  return true;
}

bool KnowrobWrapperCommunications::superclassesOfCallback(
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res)
{
  res=knowrob_wrapper.superclassesOfQuery(req);
  return true;
}


bool KnowrobWrapperCommunications::isSubSuperclassOfCallback(
  rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request& req,
  rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response& res)
{
  res=knowrob_wrapper.isSubSuperclassOfQuery(req);
  return true;
}



bool KnowrobWrapperCommunications::createInstanceCallback(
  rapp_platform_ros_communications::createInstanceSrv::Request& req,
  rapp_platform_ros_communications::createInstanceSrv::Response& res)
{
  res=knowrob_wrapper.createInstanceQuery(req);
  return true;
}

bool KnowrobWrapperCommunications::dumpOntologyCallback(
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res)
{
  res=knowrob_wrapper.dumpOntologyQuery(req);
  return true;
}

bool KnowrobWrapperCommunications::loadOntologyCallback(
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res)
{
  res=knowrob_wrapper.loadOntologyQuery(req);
  return true;
}

bool KnowrobWrapperCommunications::create_ontology_alias_callback(
  rapp_platform_ros_communications::createOntologyAliasSrv::Request& req,
  rapp_platform_ros_communications::createOntologyAliasSrv::Response& res)
{
  res=knowrob_wrapper.create_ontology_alias(req);
  return true;
}

bool KnowrobWrapperCommunications::user_instances_of_class_callback(
  rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request& req,
  rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response& res)
{
  res=knowrob_wrapper.user_instances_of_class(req);
  return true;
}

bool KnowrobWrapperCommunications::user_performance_cognitve_tests_callback(
  rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request& req,
  rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response& res)
{
  res=knowrob_wrapper.user_performance_cognitve_tests(req);
  return true;
}

bool KnowrobWrapperCommunications::create_cognitve_tests_callback(
  rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request& req,
  rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response& res)
{
  res=knowrob_wrapper.create_cognitve_tests(req);
  return true;
}

bool KnowrobWrapperCommunications::cognitive_tests_of_type_callback(
  rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request& req,
  rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response& res)
{
  res=knowrob_wrapper.cognitive_tests_of_type(req);
  return true;
}

bool KnowrobWrapperCommunications::record_user_cognitive_tests_performance_callback(
  rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request& req,
  rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response& res)
{
  res=knowrob_wrapper.record_user_cognitive_tests_performance(req);
  return true;
}

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

