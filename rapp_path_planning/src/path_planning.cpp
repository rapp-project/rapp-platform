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
               if (tf_broadcaster_pID == 0)               
               {
              ROS_DEBUG("starting tf_broadcaster_pID");
              
              execl("/opt/ros/indigo/bin/rosrun","/opt/ros/indigo/bin/rosrun","tf", "static_transform_publisher", "0.066", "1.7", "0.54", "0.49999984", "0.49960184", "0.49999984","0.50039816","map","base_link","100", (char *)0);
                }
                else if (tf_broadcaster_pID < 0)          
                {
                    std::cout << "Failed to fork tf_broadcaster_pID" << std::endl;
                    exit(1);
                    
                }
                else{
                  return true;
                }


}

bool start_map_servers(std::string node_nr_str){

      pid_t map_server_pID = fork();
         if (map_server_pID == 0)                
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
          else if (map_server_pID < 0)           
          {
              std::cout << "Failed to fork map_server node: "<<node_nr_str << std::endl;
              exit(1);
              
          }
          else{

          }

}
bool start_global_planners(std::string node_nr_str){
const char* execute_command = "/opt/ros/indigo/bin/rosparam";
 pid_t load_configs_costmap_pID = fork();
                   if (load_configs_costmap_pID == 0)                
                   {
                      ROS_DEBUG_STREAM("starting load_configs_costmap_pID for sequence: "<< node_nr_str);
                      
                      std::string robot_type = "NAO";
                      std::string costmap_file = robot_type+".yaml";
                      std::string load_configs_pkg_path = ros::package::getPath("rapp_path_planning");
                      // set yaml file path
                      std:: string execute_param_1_string = load_configs_pkg_path+"/cfg/costmap/"+costmap_file;
                      const char* execute_param_1 = execute_param_1_string.c_str();
                      ROS_DEBUG_STREAM("costmap file path:\n"<< execute_param_1_string);

                      // set params namespace
                      std:: string execute_param_2_string = "/global_planner"+node_nr_str+"/costmap/";
                      const char* execute_param_2 = execute_param_2_string.c_str();
                      //nh_.setParam("/global_planner"+node_nr_str+"/costmap/map_service", "/map_server"+node_nr_str+"/getMap");

                      execl(execute_command,execute_command,"load", execute_param_1,execute_param_2 , (char *)0);
                    }
                    else if (load_configs_costmap_pID < 0)            
                    {
                        std::cout << "Failed to fork load_configs" << std::endl;
                        return false;
                        exit(1);
                        
                    }
                    else{
                            pid_t load_configs_planner_pID = fork();
                         if (load_configs_planner_pID == 0)                
                         {
                             ROS_DEBUG_STREAM("starting load_configs_planner_pID for sequence: "<< node_nr_str);

                            std::string algorithm = "dijkstra";
                            std::string algorithm_file = algorithm+".yaml";
                            std::string load_configs_pkg_path = ros::package::getPath("rapp_path_planning");
                            // set yaml file path
                            std:: string execute_param_1_string =  load_configs_pkg_path+"/cfg/planner/"+algorithm_file;
                            const char* execute_param_1 = execute_param_1_string.c_str();
                            // set params namespace
                            std:: string execute_param_2_string = "/global_planner"+node_nr_str+"/planner/";
                            const char* execute_param_2 = execute_param_2_string.c_str();
                            execl(execute_command,execute_command,"load", execute_param_1,execute_param_2 , (char *)0);
                          }
                          else if (load_configs_planner_pID < 0)            
                          {
                              ROS_ERROR("Failed to fork load_configs");
                              return false;
                              exit(1);
                              
                          }
                          else{
                             pid_t global_planner_pID = fork();
                               if (global_planner_pID == 0)                
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
                                else if (global_planner_pID < 0)           
                                {
                              ROS_ERROR("Failed to fork global_planner");
                                    exit(1);
                                }
                                else{}
                          }
                      }

}

PathPlanning::PathPlanning(void)
{

  if(!nh_.getParam("/rapp_path_planning_plan_path_topic", pathPlanningTopic_))
  {
    ROS_WARN("Path planning topic param does not exist. Setting to: /rapp/rapp_path_planning/plan_path");
    pathPlanningTopic_ = "/rapp/rapp_path_planning/plan_path";
  }
  if(!nh_.getParam("/rapp_path_planning_threads", pathPlanningThreads_))
  {
    ROS_WARN("Path planning threads param does not exist. Setting 5 threads.");
    pathPlanningThreads_ = 5;
  }
    bool tf_status = start_tf_publisher();
if (tf_status){
    for (int node_nr=1;node_nr<pathPlanningThreads_+1;node_nr++)
    {
      std::string node_nr_str = boost::lexical_cast<std::string>(node_nr);
    start_map_servers(node_nr_str);
    start_global_planners(node_nr_str);
    //bool config_status = path_planner_.configureSequence(node_nr_str, "/home/rapp/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_map_server/maps/empty.yaml", "NAO", "dijkstra", nh_);

  }
}
if(!nh_.getParam("/rapp_path_planning_upload_map_topic", uploadMapTopic_))
  {
    ROS_ERROR("Upload map topic param does not exist. Setting to: /rapp/rapp_path_planning/upload_map");
    pathPlanningTopic_ = "/rapp/rapp_path_planning/upload_map";
  }

  // Creating the service server concerning the path planning functionality
  pathPlanningService_ = nh_.advertiseService(pathPlanningTopic_, 
    &PathPlanning::pathPlanningCallback, this);
    uploadMapService_ = nh_.advertiseService(uploadMapTopic_, 
    &PathPlanning::uploadMapCallback, this);
}


bool PathPlanning::uploadMapCallback(rapp_platform_ros_communications::MapServerUploadMapRosSrv::Request  &req,
                     rapp_platform_ros_communications::MapServerUploadMapRosSrv::Response &res){
  
  std::string seq_nr_str = path_planner_.setSequenceNR(nh_, pathPlanningThreads_);
  ros::ServiceClient upload_map_client = nh_.serviceClient<rapp_platform_ros_communications::MapServerUploadMapRosSrv>("/map_server"+seq_nr_str+"/upload_map");
  rapp_platform_ros_communications::MapServerUploadMapRosSrv upload_map_srv;
  upload_map_srv.request = req;
  if(upload_map_client.call(upload_map_srv)){
    res = upload_map_srv.response;
    return true;
  }else{
    ROS_ERROR_STREAM("FAILED to call service:\n/map_server"<< seq_nr_str << "/upload_map");
  return false; 
  }

}
bool PathPlanning::pathPlanningCallback(
  rapp_platform_ros_communications::PathPlanningRosSrv::Request& req,
  rapp_platform_ros_communications::PathPlanningRosSrv::Response& res)
{
  std::string map_path = "/home/rapp/rapp_platform_files/maps/rapp/"+req.user_name+"/"+req.map_name+".yaml";
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
 

