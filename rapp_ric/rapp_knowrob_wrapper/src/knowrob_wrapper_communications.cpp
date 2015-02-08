#include <knowrob_wrapper/knowrob_wrapper_communications.h>


KnowrobWrapperCommunications::KnowrobWrapperCommunications()
{
  ros::service::waitForService("json_prolog/query", -1);
  //ros::service::waitForService("json_prolog/query", -1);  for DB service

  if(!nh_.getParam("/ontology_subclass_of_topic", subclassesOfServiceTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }
  subclassesOfService_ = nh_.advertiseService(subclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::subclassesOfCallback, this);

  if(!nh_.getParam("/ontology_superclass_of_topic", superclassesOfServiceTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }      
  superclassesOfService_ = nh_.advertiseService(superclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::superclassesOfCallback, this);
    
  if(!nh_.getParam("/ontology_create_instance_topic", createInstanceServiceTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }      
  createInstanceService_ = nh_.advertiseService(createInstanceServiceTopic_,
    &KnowrobWrapperCommunications::createInstanceCallback, this);    

  if(!nh_.getParam("/ontology_dump_ontology_topic", dumpOntologyServiceTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }         
  dumpOntologyService_ = nh_.advertiseService(dumpOntologyServiceTopic_,
    &KnowrobWrapperCommunications::dumpOntologyCallback, this);    

  if(!nh_.getParam("/ontology_load_ontology_topic", loadOntologyServiceTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }  
  loadOntologyService_ = nh_.advertiseService(loadOntologyServiceTopic_,
    &KnowrobWrapperCommunications::loadOntologyCallback, this);  

  if(!nh_.getParam("/ontology_user_instances_from_class_topic", userInstancesFromClassServiceTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }   
  userInstancesFromClassServiceTopic_ = "ric/knowrob/userInstancesFromClass";
  userInstancesFromClassService_ = nh_.advertiseService(userInstancesFromClassServiceTopic_,
    &KnowrobWrapperCommunications::userInstancesFromClassCallback, this);  
    
    

  ROS_WARN("KnowRob ROS wrapper initialized");
}

bool KnowrobWrapperCommunications::subclassesOfCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  std::vector<std::string> res_ = 
    knowrob_wrapper.subclassesOfQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  return true;
}

bool KnowrobWrapperCommunications::superclassesOfCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  std::vector<std::string> res_ = 
    knowrob_wrapper.superclassesOfQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  return true;
}

bool KnowrobWrapperCommunications::createInstanceCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  std::vector<std::string> res_ = 
    knowrob_wrapper.createInstanceQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  return true;
}

bool KnowrobWrapperCommunications::dumpOntologyCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  std::vector<std::string> res_ = 
    knowrob_wrapper.dumpOntologyQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  return true;
}

bool KnowrobWrapperCommunications::loadOntologyCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  std::vector<std::string> res_ = 
    knowrob_wrapper.loadOntologyQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  return true;
}

bool KnowrobWrapperCommunications::userInstancesFromClassCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  //std::cout<<req.return_cols[0].data;
  std::vector<std::string> res_ = 
    knowrob_wrapper.userInstancesFromClassQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  
  return true;
}
