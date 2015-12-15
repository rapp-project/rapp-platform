#ifndef RAPP_FACE_DETECTION_NODE
#define RAPP_FACE_DETECTION_NODE

#include "ros/ros.h"
#include "ros/package.h"
#include <navfn/MakeNavPlan.h>
#include <navfn/MakeNavPlanResponse.h>
#include <rapp_platform_ros_communications/PathPlanningRosSrv.h>
#include <signal.h>
#include <path_planning/path_planner.h>
//converting variables
#include <boost/lexical_cast.hpp>
// for service manager -> determine if service is active
#include <ros/service_manager.h>

class PathPlanning
{
  public:

    // Default constructor
    PathPlanning(void);

    bool pathPlanningCallback(
      rapp_platform_ros_communications::PathPlanningRosSrv::Request& req,
      rapp_platform_ros_communications::PathPlanningRosSrv::Response& res
      );

  private:
    // The ROS node handle
    ros::NodeHandle nh_;

    // The service server 
    ros::ServiceServer pathPlanningService_;
    std::vector<ros::ServiceServer> pathPlanningThreadServices_;

    // Topic nomeclarure
    std::string pathPlanningTopic_;
    int pathPlanningThreads_;
    
    PathPlanner path_planner_; 
};

#endif
