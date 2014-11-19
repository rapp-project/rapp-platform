#include <knowrob_wrapper/knowrob_wrapper.h>

#include <rapp_platform_ros_communications/OntologySimpleQuerySrv.h>

#include <ros/ros.h>

class KnowrobWrapperCommunications
{
  private:
    ros::NodeHandle nh_;
    
    ros::ServiceServer subclassOfService_;
    
    std::string subclassOfServiceTopic_;
    
    KnowrobWrapper knowrob_wrapper;

  public:

    KnowrobWrapperCommunications();

    bool subclassOfCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);

};
