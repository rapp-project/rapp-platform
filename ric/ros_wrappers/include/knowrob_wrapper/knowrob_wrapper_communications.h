#include <knowrob_wrapper/knowrob_wrapper.h>

#include <rapp_platform_ros_communications/OntologySimpleQuerySrv.h>

#include <rapp_platform_ros_communications/DbWrapperSrv.h>

#include <rapp_platform_ros_communications/StringArrayMsg.h>

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
    
    ros::ServiceServer createInstanceService_;    
    std::string createInstanceServiceTopic_;
    
    ros::ServiceServer dumpOntologyService_;    
    std::string dumpOntologyServiceTopic_;
    
    ros::ServiceServer loadOntologyService_;    
    std::string loadOntologyServiceTopic_;
    
    ros::ServiceServer userInstancesFromClassService_;    
    std::string userInstancesFromClassServiceTopic_;    
    //ros::ServiceServer instanceFromClassService_;    
    //std::string instanceFromClassServiceTopic_;
    
    

  public:

    KnowrobWrapperCommunications();

    bool subclassesOfCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
      
    bool superclassesOfCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
      
    bool createInstanceCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
      
    bool dumpOntologyCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
      
    bool loadOntologyCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
      
    bool userInstancesFromClassCallback(
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Request& req,
      rapp_platform_ros_communications::OntologySimpleQuerySrv::Response& res);
};
