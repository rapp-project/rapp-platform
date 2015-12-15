#include <path_planning/path_planning.h>
#include <iostream>
#include <string>

// Required by for routine
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <stdlib.h>   // Declaration for exit()
#include <boost/filesystem.hpp> // find_file

bool input_poses_correct(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal){

//ToDo
  //  check if pose coordinate is grater then map size

 bool status = false;
 if (start.pose.position.x != 0){
 if (goal.pose.position.x != 0){
 if (goal.pose.position.y != 0){
 if (start.pose.position.y != 0){

status = true;

}}}}

return status;

}
bool exists_file (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

bool  start_tf_publisher(){
    pid_t tf_broadcaster_pID = fork();
               if (tf_broadcaster_pID == 0)                // child
               {
              ROS_DEBUG("starting tf_broadcaster_pID");
              
              execl("/opt/ros/indigo/bin/rosrun","/opt/ros/indigo/bin/rosrun","tf", "static_transform_publisher", "0.066", "1.7", "0.54", "0.49999984", "0.49960184", "0.49999984","0.50039816","map","base_link","100", (char *)0);
                }
                else if (tf_broadcaster_pID < 0)            // failed to fork
                {
                    std::cout << "Failed to fork tf_broadcaster_pID" << std::endl;
                    exit(1);
                    // Throw exception
                }
                else{
                  return true;
                }


}

bool start_map_servers(std::string node_nr_str){

      pid_t map_server_pID = fork();
         if (map_server_pID == 0)                // child
         {
            ROS_DEBUG_STREAM("starting map_server node: "<< node_nr_str);
 
            std:: string execute_param_1_string = "__name:=map_server"+node_nr_str;
            const char* execute_param_1 = execute_param_1_string.c_str();
                      ROS_DEBUG_STREAM("map_server name:\n" << node_nr_str );

            // set map path
            std:: string execute_param_2_string = "/home/rapp/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_map_server/maps/empty.yaml";
            const char* execute_param_2 = execute_param_2_string.c_str();
            // remap map publication topic
            std:: string execute_param_3_string = "/map:=/map_server"+node_nr_str+"/map";
            const char* execute_param_3 = execute_param_3_string.c_str();
            execl("/opt/ros/indigo/bin/rosrun", "/opt/ros/indigo/bin/rosrun", "rapp_map_server", "rapp_map_server", execute_param_1, execute_param_2, execute_param_3, (char *)0);
          }
          else if (map_server_pID < 0)            // failed to fork
          {
              std::cout << "Failed to fork map_server node: "<<node_nr_str << std::endl;
              exit(1);
              // Throw exception
          }
          else{

          }

}
bool start_global_planners(std::string node_nr_str){

     pid_t global_planner_pID = fork();
       if (global_planner_pID == 0)                // child
       {
            ROS_DEBUG_STREAM("starting global_planner node: "<< node_nr_str);

          // ROS node name
          std:: string execute_param_1_string = "__name:=global_planner"+node_nr_str;
          const char* execute_param_1 = execute_param_1_string.c_str();
          // remap map subscribtion topic
          std:: string execute_param_2_string =  "/map:=/map_server"+node_nr_str+"/map";
          const char* execute_param_2 = execute_param_2_string.c_str();

          execl("/opt/ros/indigo/bin/rosrun","/opt/ros/indigo/bin/rosrun","global_planner", "planner", execute_param_1,  execute_param_2, (char *)0);

        }
        else if (global_planner_pID < 0)            // failed to fork
        {
      ROS_ERROR("Failed to fork global_planner");
            exit(1);
            // Throw exception
        }
        else{}
  

}

PathPlanning::PathPlanning(void)
{

  if(!nh_.getParam("/rapp_path_planning_plan_path_topic", pathPlanningTopic_))
  {
    ROS_ERROR("Path planning topic param does not exist. Setting to: /rapp/rapp_path_planning/plan_path");
    pathPlanningTopic_ = "/rapp/rapp_path_planning/plan_path";
  }
  if(!nh_.getParam("/rapp_path_planning_threads", pathPlanningThreads_))
  {
    ROS_ERROR("Path planning threads param does not exist. Setting 5 threads.");
    pathPlanningThreads_ = 5;
  }
    bool tf_status = start_tf_publisher();
if (tf_status){
    for (int node_nr=1;node_nr<pathPlanningThreads_+1;node_nr++)
    {
      std::string node_nr_str = boost::lexical_cast<std::string>(node_nr);
    start_map_servers(node_nr_str);
    bool config_status = path_planner_.configureSequence(node_nr_str, "/home/rapp/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_map_server/maps/empty.yaml", "NAO", "dijkstra", nh_);
    start_global_planners(node_nr_str);
  }
}
  // Creating the service server concerning the path planning functionality
  pathPlanningService_ = nh_.advertiseService(pathPlanningTopic_, 
    &PathPlanning::pathPlanningCallback, this);
}



bool PathPlanning::pathPlanningCallback(
  rapp_platform_ros_communications::PathPlanningRosSrv::Request& req,
  rapp_platform_ros_communications::PathPlanningRosSrv::Response& res)
{
  std::string map_path = ros::package::getPath("rapp_map_server")+"/maps/"+req.map_name+".yaml";
  std::string costmap_file_path = ros::package::getPath("rapp_path_planning")+"/cfg/costmap/"+req.robot_type+".yaml";
  std::string algorithm_file_path = ros::package::getPath("rapp_path_planning")+"/cfg/planner/"+req.algorithm+".yaml";
  navfn::MakeNavPlanResponse response;
  if (input_poses_correct(req.start, req.goal)){
    if (exists_file(algorithm_file_path)){
      if (exists_file(costmap_file_path)){
        if (exists_file(map_path)){
           ROS_DEBUG("NEW <<Path_planning>> SERVICE STARTED");
            std::string seq_nr_str = path_planner_.setSequenceNR(nh_, pathPlanningThreads_);
            ROS_DEBUG_STREAM("SEQ-NR is: " << seq_nr_str);
            bool config_status = path_planner_.configureSequence(seq_nr_str, map_path, req.robot_type, req.algorithm, nh_);

            response = path_planner_.startSequence(seq_nr_str, req.start, req.goal, nh_);
           
            // old 
            //response = path_planner_.plannPath(req.map_path, req.robot_type, req.algorithm, req.start, req.goal, nh_);
            res.plan_found =  response.plan_found;
            
            res.error_message = response.error_message;
            res.path  = response.path;
            nh_.setParam("/rapp/rapp_path_planning/seq_"+seq_nr_str+"/busy", false);
        }else{
            res.plan_found = 2;
            res.error_message = "Input map does not exist";
            ROS_ERROR("Input map does not exist");
        }
      }else{
          res.plan_found = 3;
          res.error_message = "Input robot_type does not exist";

          ROS_ERROR("Input robot_type does not exist");
      }
    }else{
        res.plan_found = 4;
        res.error_message = "Input algorithm does not exist";

        ROS_ERROR("Input algorithm does not exist");
    }
  }else{
        res.plan_found = 5;
        res.error_message = "Robot pose can NOT be placed at the map border";

        ROS_ERROR("Robot pose can NOT be placed at the map border");
  }
}
 

                
  // ros::ServiceClient client = nh_.serviceClient<rapp_platform_ros_communications::PathPlanningRosSrv>("/rapp_path_planning_node/plann_path");
  // rapp_platform_ros_communications::PathPlanningRosSrv srv;
  // srv.request.algorithm = req.algorithm;
  // srv.request.robot_type = req.robot_type;
  // srv.request.map_path = req.map_path;
  // srv.request.start = req.start;
  // srv.request.finish = req.finish;
  // if (client.call(srv))
  // {
  //   res.path = srv.response.path);
  //   return true;

  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service rapp_path_planning_service_handler");
  //   return false;
  // }

