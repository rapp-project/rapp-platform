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

bool KnowrobWrapperCommunications::user_instances_of_class_callback(
  rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request& req,
  rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response& res)
{
  res=knowrob_wrapper.user_instances_of_class(req);
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

