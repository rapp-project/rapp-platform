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
  ros::service::waitForService("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesWriteData", -1);
  ros::service::waitForService("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesFetchData", -1);
  //ros::service::waitForService("json_prolog/query", -1);  for DB service

  if(!nh_.getParam("/ontology_subclass_of_topic", subclassesOfServiceTopic_))
  {
    ROS_ERROR("ontology_subclass_of_topic");
  }
  subclassesOfService_ = nh_.advertiseService(subclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::subclassesOfCallback, this);

  if(!nh_.getParam("/ontology_superclass_of_topic", superclassesOfServiceTopic_))
  {
    ROS_ERROR("ontology_superclass_of_topic");
  }      
  superclassesOfService_ = nh_.advertiseService(superclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::superclassesOfCallback, this);
    
  if(!nh_.getParam("/ontology_create_instance_topic", createInstanceServiceTopic_))
  {
    ROS_ERROR("ontology_create_instance_topic");
  }      
  createInstanceService_ = nh_.advertiseService(createInstanceServiceTopic_,
    &KnowrobWrapperCommunications::createInstanceCallback, this);    

  if(!nh_.getParam("/ontology_dump_ontology_topic", dumpOntologyServiceTopic_))
  {
    ROS_ERROR("ontology_dump_ontology_topic");
  }         
  dumpOntologyService_ = nh_.advertiseService(dumpOntologyServiceTopic_,
    &KnowrobWrapperCommunications::dumpOntologyCallback, this);    

  if(!nh_.getParam("/ontology_load_ontology_topic", loadOntologyServiceTopic_))
  {
    ROS_ERROR("ontology_load_ontology_topic");
  }  
  loadOntologyService_ = nh_.advertiseService(loadOntologyServiceTopic_,
    &KnowrobWrapperCommunications::loadOntologyCallback, this);  

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
    
    

  ROS_WARN("KnowRob ROS wrapper initialized");
}

bool KnowrobWrapperCommunications::subclassesOfCallback(
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res)
{
  //std::vector<std::string> res_ = 
    //knowrob_wrapper.subclassesOfQuery(req.query_term.data);
  //for(unsigned int i = 0 ; i < res_.size() ; i++)
  //{
    //std_msgs::String s;
    //s.data = res_[i];
    //res.results.push_back(s);
  //}
  res=knowrob_wrapper.subclassesOfQuery(req);
  return true;
}

bool KnowrobWrapperCommunications::superclassesOfCallback(
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request& req,
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response& res)
{
  //std::vector<std::string> res_ = 
    //knowrob_wrapper.superclassesOfQuery(req.query_term.data);
  //for(unsigned int i = 0 ; i < res_.size() ; i++)
  //{
    //std_msgs::String s;
    //s.data = res_[i];
    //res.results.push_back(s);
  //}
  res=knowrob_wrapper.superclassesOfQuery(req);
  return true;
}

bool KnowrobWrapperCommunications::createInstanceCallback(
  rapp_platform_ros_communications::createInstanceSrv::Request& req,
  rapp_platform_ros_communications::createInstanceSrv::Response& res)
{
  //std::vector<std::string> res_ = 
    res=knowrob_wrapper.createInstanceQuery(req);
  //for(unsigned int i = 0 ; i < res_.size() ; i++)
  //{
    //std_msgs::String s;
    //s.data = res_[i];
    //res.results.push_back(s);
  //}
  //res.success = true;
  return true;
}

bool KnowrobWrapperCommunications::dumpOntologyCallback(
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res)
{
  //std::vector<std::string> res_ = 
    //knowrob_wrapper.dumpOntologyQuery(req.query_term.data);
  //for(unsigned int i = 0 ; i < res_.size() ; i++)
  //{
    //std_msgs::String s;
    //s.data = res_[i];
    //res.results.push_back(s);
  //}
  res=knowrob_wrapper.dumpOntologyQuery(req);
  return true;
}

bool KnowrobWrapperCommunications::loadOntologyCallback(
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request& req,
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response& res)
{
  //std::vector<std::string> res_ = 
    //knowrob_wrapper.loadOntologyQuery(req.query_term.data);
  //for(unsigned int i = 0 ; i < res_.size() ; i++)
  //{
    //std_msgs::String s;
    //s.data = res_[i];
    //res.results.push_back(s);
  //}
  res=knowrob_wrapper.loadOntologyQuery(req);
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
