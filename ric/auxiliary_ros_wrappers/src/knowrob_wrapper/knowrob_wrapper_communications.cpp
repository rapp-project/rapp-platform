#include <knowrob_wrapper/knowrob_wrapper_communications.h>

KnowrobWrapperCommunications::KnowrobWrapperCommunications()
{
  subclassOfServiceTopic_ = "ric/knowrob/subclass_of";

  subclassOfService_ = nh_.advertiseService(subclassOfServiceTopic_,
    &KnowrobWrapperCommunications::subclassOfCallback, this);
}

bool KnowrobWrapperCommunications::subclassOfCallback(
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
  rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res)
{
  std::vector<std::string> res_ = 
    knowrob_wrapper.subclassOfQuery(req.query_term.data);
  for(unsigned int i = 0 ; i < res_.size() ; i++)
  {
    std_msgs::String s;
    s.data = res_[i];
    res.results.push_back(s);
  }
  return true;
}


