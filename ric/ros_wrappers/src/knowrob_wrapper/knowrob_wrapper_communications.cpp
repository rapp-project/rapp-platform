#include <knowrob_wrapper/knowrob_wrapper_communications.h>


KnowrobWrapperCommunications::KnowrobWrapperCommunications()
{
  ros::service::waitForService("json_prolog/query", -1);
  //ros::service::waitForService("json_prolog/query", -1);  for DB service
  

  subclassesOfServiceTopic_ = "ric/knowrob/subclasses_of";
  subclassesOfService_ = nh_.advertiseService(subclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::subclassesOfCallback, this);
    
  superclassesOfServiceTopic_ = "ric/knowrob/superclasses_of";
  superclassesOfService_ = nh_.advertiseService(superclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::superclassesOfCallback, this);    
    
  createInstanceServiceTopic_ = "ric/knowrob/createInstance";
  createInstanceService_ = nh_.advertiseService(createInstanceServiceTopic_,
    &KnowrobWrapperCommunications::createInstanceCallback, this);    
    
  dumpOntologyServiceTopic_ = "ric/knowrob/dumpOntology";
  dumpOntologyService_ = nh_.advertiseService(dumpOntologyServiceTopic_,
    &KnowrobWrapperCommunications::dumpOntologyCallback, this);    
    
  loadOntologyServiceTopic_ = "ric/knowrob/loadOntology";
  loadOntologyService_ = nh_.advertiseService(loadOntologyServiceTopic_,
    &KnowrobWrapperCommunications::loadOntologyCallback, this);  

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
