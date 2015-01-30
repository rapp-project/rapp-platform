#include <knowrob_wrapper/knowrob_wrapper.h>

#include <rapp_platform_ros_communications/OntologySimpleQuerySrv.h>

#include <ros/ros.h>

class KnowrobWrapperCommunications
{
  private:
    ros::NodeHandle nh_;    
    KnowrobWrapper knowrob_wrapper;
    
    ros::ServiceServer subclassesOfService_;    
    std::string subclassesOfServiceTopic_;
    
    ros::ServiceServer superclassesOfService_;    
    std::string superclassesOfServiceTopic_;
    
    ros::ServiceServer instanceFromClassService_;    
    std::string instanceFromClassServiceTopic_;
    
    ros::ServiceServer instanceFromClassService_;    
    std::string instanceFromClassServiceTopic_;
    
    

  public:

    KnowrobWrapperCommunications();

    bool subclassesOfCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
      
    bool superclassesOfCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
      
    bool instanceFromClassCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
};
