#ifndef RAPP_PATH_PLANNER_NODE
#define RAPP_PATH_PLANNER_NODE

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

/**
 * @class PathPlanner
 * @brief Class that implements a path planning algorithm based on global_planner and map_server ROS nodes
 */
class PathPlanner
{
  public:
    
    /** 
     * @brief   Default constructor
    */
    PathPlanner(void);

    /** 
     * @brief   Determines next planning seqence (which global_planner and map_server should be used)
     * @param   &nh_ [ros::NodeHandle] node handler for getParam() method,
     * @param   pathPlanningThreads_ [int] Number of planning sequences,
     * @return  [std::string] Next sequence nr.

    */
    std::string setSequenceNR(ros::NodeHandle &nh_, int pathPlanningThreads_);
     /** 
     * @brief   Configures sequence. Sets map, approprate costmap parameters for specified robot type, sets global_planner to use detemined algorithm.
     * @param   seq_nr [std::string] ID of current sequence,
     * @param   map_name [std::string] Name of map that should be passed to global_planner,
     * @param   robot_type [std::string] Name of robot_type. It is used to configure costmap,
     * @param   algorithm [std::string] Name of algorithm that should be used by global_planner,
     * @param   &nh_ [ros::NodeHandle] node handler for getParam() method,
     * @return  [bool] Returns if sequence was configured correctly. Returns false if PathPlanner could not configure sequence in 5 sec.

    */  
    bool configureSequence(std::string seq_nr, std::string map_name, std::string robot_type, std::string algorithm, ros::NodeHandle &nh_);

    /**
     * @brief   Starts planning service of determined planning sequence.
     * @param   start [geometry_msgs::PoseStamped] Robot start pose,
     * @param   finish [geometry_msgs::PoseStamped] Robot goal pose.
     * @return  [navfn::MakeNavPlanResponse]: 
                            -> plan_found:
                                    * 0 : path cannot be planned.
                                    * 1 : path found 
                                    * 2 : wrong map name
                                    * 3 : wrong robot type
                                    * 4 : wrong algorithm
                            -> error_message : error explenation
                            -> path : if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal vector of PoseStamped objects
     */

    navfn::MakeNavPlanResponse startSequence(std::string seq_nr, geometry_msgs::PoseStamped request_start, geometry_msgs::PoseStamped request_goal, ros::NodeHandle &nh_);

//////
//
//  OLD APPROACH
//
/////

    /**
     * @brief   Returns planned path. Path is planned using global_planner and map_server nodes.
     * @param   map_path [std::string] Name of desired map,
     * @param   robot [std::string] Type of a robot. It is used for approriate cosmap configuration,
     * @param   algorithm [std::string] Path planning algorithm specification,
     * @param   start [geometry_msgs::PoseStamped] Path start pose,
     * @param   finish [geometry_msgs::PoseStamped] Path goal pose.
     * @return  [navfn::MakeNavPlanResponse]: 
                            -> plan_found:
                                    * 0 : path cannot be planned.
                                    * 1 : path found 
                                    * 2 : wrong map name
                                    * 3 : wrong robot type
                                    * 4 : wrong algorithm
                            -> error_message : error explenation
                            -> path : vector of PoseStamped objects
     



   


    navfn::MakeNavPlanResponse plannPath(std::string map_path, std::string robot, std::string algorithm, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, ros::NodeHandle& nh);
*/
      private:


};

#endif

