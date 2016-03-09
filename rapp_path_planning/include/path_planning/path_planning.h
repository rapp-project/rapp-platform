#ifndef RAPP_PATH_PLANNING_NODE
#define RAPP_PATH_PLANNING_NODE

#include "ros/ros.h"
#include "ros/package.h"
#include <navfn/MakeNavPlan.h>
#include <navfn/MakeNavPlanResponse.h>
#include <rapp_platform_ros_communications/PathPlanningRosSrv.h>
#include "rapp_platform_ros_communications/MapServerGetMapRosSrv.h"
#include "rapp_platform_ros_communications/Costmap2dRosSrv.h"
#include "rapp_platform_ros_communications/MapServerUploadMapRosSrv.h"
#include <signal.h>
#include <path_planning/path_planner.h>
//converting variables
#include <boost/lexical_cast.hpp>
// for service manager -> determine if service is active
#include <ros/service_manager.h>
//get rapp-platform home directory
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

class PathPlanning
{
  public:

    /** 
     * @brief   Default constructor
    */
    PathPlanning(void);
    ~PathPlanning(void);
    /** 
     * @brief   Upload map to RAPP Platform
     * @param   &req: [std::string] user_name, [std::string] map_name, [float32] resolution, [float32[]] origin, int16 negate, [float32] occupied_thresh, [float32] free_thresh, [uint32] file_size, [char[]] data,
     * @param   &res: [byte] status ,
     * @return  [bool] true-> success, false-> failure.

    */    
    bool uploadMapCallback(rapp_platform_ros_communications::MapServerUploadMapRosSrv::Request  &req,
                     rapp_platform_ros_communications::MapServerUploadMapRosSrv::Response &res);
    /** 
     * @brief   Determines next planning seqence (which global_planner and map_server should be used)
     * @param   [navfn::MakeNavPlanRequest] &req: 
                  * [std::string] req.user_name - Determine user name - owner of the environment map 
                  * [std::string] req.map_name - Contains path to the desired map
                  * [std::string] req.robot_type - Contains type of the robot. It is required to determine it's parameters (footprint etc.)
                  * [std::string] req.algorithm - Contains path planning algorithm name
                  * [geometry_msgs/PoseStamped] req.start - Contains start pose of the robot
                  * [geometry_msgs/PoseStamped] req.goal - Contains goal pose of the robot
     * @param  [navfn::MakeNavPlanResponse] &res:
                  * [uint8] res.plan_found:
                      * 0 : path cannot be planned.
                      * 1 : path found 
                      * 2 : wrong map name
                      * 3 : wrong robot type
                      * 4 : wrong algorithm
                  * [std::string] res.error_message : error explenation
                  * [geometry_msgs/PoseStamped[]] res.path : if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal vector of PoseStamped objects
    **/   
    bool pathPlanningCallback(
      rapp_platform_ros_communications::PathPlanningRosSrv::Request& req,
      rapp_platform_ros_communications::PathPlanningRosSrv::Response& res
      );

  private:
    // The ROS node handle
    ros::NodeHandle nh_;
    // RAPP-platform home_dir
    const char *homedir;
    // The service server 
    ros::ServiceServer pathPlanningService_;
    ros::ServiceServer uploadMapService_;
    std::vector<ros::ServiceServer> pathPlanningThreadServices_;
    std::vector<pid_t> GP_pIDs;
    std::vector<pid_t> MS_pIDs;
    pid_t TP_pID;
    // Topic nomeclarure
    std::string pathPlanningTopic_;
    std::string uploadMapTopic_;
    int pathPlanningThreads_;
    
    PathPlanner path_planner_; 
};

#endif
