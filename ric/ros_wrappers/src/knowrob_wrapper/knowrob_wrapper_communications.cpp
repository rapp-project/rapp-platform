#include <knowrob_wrapper/knowrob_wrapper_communications.h>

KnowrobWrapperCommunications::KnowrobWrapperCommunications()
{
  ros::service::waitForService("json_prolog/query", -1);
  

  subclassesOfServiceTopic_ = "ric/knowrob/subclasses_of";
  subclassesOfService_ = nh_.advertiseService(subclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::subclassesOfCallback, this);
    
  superclassesOfServiceTopic_ = "ric/knowrob/superclasses_of";
  superclassesOfService_ = nh_.advertiseService(superclassesOfServiceTopic_,
    &KnowrobWrapperCommunications::superclassesOfCallback, this);    
    
  instanceFromClassServiceTopic_ = "ric/knowrob/instanceFromClass";
  instanceFromClassService_ = nh_.advertiseService(instanceFromClassServiceTopic_,
    &KnowrobWrapperCommunications::instanceFromClassCallback, this);    
    
    
    
    
    

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

bool KnowrobWrapperCommunications::instanceFromClassCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  std::vector<std::string> res_ = 
    knowrob_wrapper.instanceFromClassQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  return true;
}
