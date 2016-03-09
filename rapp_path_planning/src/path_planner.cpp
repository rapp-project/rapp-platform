#include <path_planning/path_planning.h>


PathPlanner::PathPlanner(void)
{
}

// set next planning sequence ID
std::string PathPlanner::setSequenceNR(ros::NodeHandle &nh_, int pathPlanningThreads_){
    int seq_nr_int=1;
    std::string seq_nr_str;
    if (nh_.hasParam("/rapp/rapp_path_planning/last_seq")){
        nh_.getParam("/rapp/rapp_path_planning/last_seq", seq_nr_int);
       
         seq_nr_int++;
       if (seq_nr_int > pathPlanningThreads_)
        seq_nr_int = 1;
    }

    seq_nr_str = boost::lexical_cast<std::string>(seq_nr_int);
    nh_.setParam("/rapp/rapp_path_planning/last_seq", seq_nr_int);

    return seq_nr_str;
}
// configure sequence -> load .yaml files and load proper map
bool PathPlanner::configureSequence(std::string seq_nr, std::string map_path, std::string robot_type, std::string algorithm, ros::NodeHandle &nh_){

  nh_.setParam("/map_server"+seq_nr+"/setMap", map_path);
  ros::ServiceClient get_map_client = nh_.serviceClient<rapp_platform_ros_communications::MapServerGetMapRosSrv>("map_server"+seq_nr+"/get_map");
  ros::ServiceClient set_costmap_client = nh_.serviceClient<rapp_platform_ros_communications::Costmap2dRosSrv>("/global_planner"+seq_nr+"/costmap_map_update");
  
  rapp_platform_ros_communications::MapServerGetMapRosSrv get_map_srv;
  rapp_platform_ros_communications::Costmap2dRosSrv set_costmap_srv;
get_map_srv.request.map_path = map_path;
uint32_t serv_port;
  std::string serv_node_name, serv_name;
  ros::ServiceManager srv_manager; 
 bool serv_active = srv_manager.lookupService("map_server"+seq_nr+"/get_map",serv_node_name,serv_port);

  while (!serv_active){
  ROS_WARN_STREAM("Waiting for:"<<" map_server"<<seq_nr<<"/get_map" << " service ");
  serv_active = srv_manager.lookupService("map_server"+seq_nr+"/get_map",serv_node_name,serv_port);
  ros::Duration(1).sleep();
  }


  if (get_map_client.call(get_map_srv))
  { 
    set_costmap_srv.request.map = get_map_srv.response.map;

    if (set_costmap_client.call(set_costmap_srv))
    { 
      ROS_INFO_STREAM("Costmap update for SEQ: " << seq_nr);

    }else{
      ROS_ERROR_STREAM("Costmap update error for SEQ: " << seq_nr  << "\n path planner cannot set new costmap");

    }
  }else{

    ROS_ERROR_STREAM("Costmap update error for SEQ: " << seq_nr << "\n path planner cannot get map from map server");

  }
 const char* execute_command = "/opt/ros/indigo/bin/rosparam";
 pid_t load_configs_costmap_pID = fork();
                   if (load_configs_costmap_pID == 0)                
                   {
                      ROS_DEBUG_STREAM("starting load_configs_costmap_pID for sequence: "<< seq_nr);
                      
                      //robot_type = "NAO";
                      std::string costmap_file = robot_type+".yaml";
                      std::string load_configs_pkg_path = ros::package::getPath("rapp_path_planning");
                      // set yaml file path
                      std:: string execute_param_1_string = load_configs_pkg_path+"/cfg/costmap/"+costmap_file;
                      const char* execute_param_1 = execute_param_1_string.c_str();
                      ROS_DEBUG_STREAM("costmap file path:\n"<< execute_param_1_string);

                      // set params namespace
                      std:: string execute_param_2_string = "/global_planner"+seq_nr+"/costmap/";
                      const char* execute_param_2 = execute_param_2_string.c_str();
                      nh_.setParam("/global_planner"+seq_nr+"/costmap/map_service", "/map_server"+seq_nr+"/getMap");

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
                             ROS_DEBUG_STREAM("starting load_configs_planner_pID for sequence: "<< seq_nr);

                            //algorithm = "dijkstra";
                            std::string algorithm_file = algorithm+".yaml";
                            std::string load_configs_pkg_path = ros::package::getPath("rapp_path_planning");
                            // set yaml file path
                            std:: string execute_param_1_string =  load_configs_pkg_path+"/cfg/planner/"+algorithm_file;
                            const char* execute_param_1 = execute_param_1_string.c_str();
                            // set params namespace
                            std:: string execute_param_2_string = "/global_planner"+seq_nr+"/planner/";
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
                            
                            std:: string costmap_param_name = "/global_planner"+seq_nr+"/costmap/map_topic";

                            std:: string planner_param_name = "/global_planner"+seq_nr+"/planner/use_dijkstra";
                            ros::Time time_start = ros::Time::now();
                            ros::Time time_now;

                            while (!nh_.hasParam(costmap_param_name) && !nh_.hasParam(planner_param_name)){
                              ros::Duration(0.1).sleep();
                              time_now = ros::Time::now();
                              if (time_now > time_start + ros::Duration(5))
                                return false;
                            }
                            return true;
                          }
  
                        }
}
// send request to approprate global_planner and return MakeNavPlanResponse 
navfn::MakeNavPlanResponse PathPlanner::startSequence(std::string seq_nr, geometry_msgs::PoseStamped request_start, geometry_msgs::PoseStamped request_goal, ros::NodeHandle &nh_){
  navfn::MakeNavPlanResponse planned_path; 
uint32_t serv_port;
  std::string serv_node_name, serv_name;
  serv_name = "/global_planner"+seq_nr+"/make_plan";
  ros::ServiceManager srv_manager; 
  bool serv_active = srv_manager.lookupService(serv_name,serv_node_name,serv_port);

  ros::Duration(1.5).sleep();

  while (!serv_active){
  ROS_ERROR("Can't find service make_plan");
  serv_active = srv_manager.lookupService(serv_name,serv_node_name,serv_port);
  ros::Duration(1).sleep();
  }
   ROS_DEBUG_STREAM("HOST name:\n" << serv_node_name << "service port:\n" << serv_port);
   
  ros::ServiceClient client = nh_.serviceClient<navfn::MakeNavPlan>("/global_planner"+seq_nr+"/make_plan");
  navfn::MakeNavPlan srv;
  srv.request.start = request_start;
  srv.request.goal = request_goal;
  
  if (client.call(srv))
  {
        navfn::MakeNavPlanResponse planned_path; 
        planned_path = srv.response;
        ROS_DEBUG("Path planning service ended");
        return planned_path;

  }
  
  else
  {
    ROS_ERROR("Failed to call service make_plan");
    return planned_path;
  }
                  
}


//////
//
//  OLD APPROACH
//
/////
// bool deleteParameters(std::string &new_nodes_string_id, ros::NodeHandle &nh_){
//   ROS_ERROR("deleting params\n");
//   ROS_DEBUG_STREAM(new_nodes_string_id);
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/allow_unknown");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/default_tolerance");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/visualize_potential");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/use_dijkstra");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/use_quadratic");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/use_grid_path");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/old_navfn_behavior");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/cost_factor");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/lethal_cost");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/neutral_cost");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/orientation_mode");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/planner/publish_potential");

// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/global_frame");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/robot_base_frame");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/update_frequency");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/publish_frequency");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/plugins");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/publish_voxel_map");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/static_map");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/map_topic");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/unknown_cost_value");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/lethal_cost_threshold");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/footprint");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/footprint_padding");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/cost_scaling_factor");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/height");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/origin_x");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/origin_y");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/resolution");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/robot_radius");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/transform_tolerance");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/width");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/inflater/cost_scaling_factor");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/inflater/enabled");
// nh_.deleteParam("/global_planner"+new_nodes_string_id+"/costmap/inflater/inflation_radius");
// return true;

// }
// navfn::MakeNavPlanResponse PathPlanner::plannPath(std::string map_path, std::string  robot, std::string  algorithm, geometry_msgs::PoseStamped request_start, geometry_msgs::PoseStamped request_goal, ros::NodeHandle &nh_)
// {
 

//   ros::package::V_string nodes;
//   ros::master::getNodes(nodes);
//   std::string pathPlanningTopic_;
//   int pathPlanningThreads_;
//   nh_.getParam("/rapp_path_planning_threads", pathPlanningThreads_);
//   // initialize examplary command for execl instructions
//   const char* execute_command = "";
//   const char* execute_pkg_name = "";
//   const char* execute_pkg_exe_name = "";

//   // determine individual new_nodes_ID
//   int new_nodes_ID = 1;
//   std::string new_node_name = "/map_server";
//   std::string new_nodes_string_id = "empty";
//   ROS_DEBUG_STREAM( "looking for launched nodes\n  Number of avaliable threads:\n"<<pathPlanningThreads_);
//   for (int i=0; i < nodes.size();i++){
//           ROS_DEBUG_STREAM("\nNodes: " << nodes.at(i));
//   }
//   for (int i=0; i < pathPlanningThreads_;i++){
//       new_nodes_string_id = boost::lexical_cast<std::string>(new_nodes_ID);//std::to_string(new_nodes_ID);
//       new_node_name = new_node_name + new_nodes_string_id;
//                                 // old method -> check if /map_server node exist
//                                 // if (std::find(nodes.begin(), nodes.end(), new_node_name) != nodes.end() ){
//                                 //     new_nodes_ID++;
//                                 //     ROS_DEBUG_STREAM("Node: " << new_node_name << "exist");
//                                 // }else{
//                                 //   ROS_DEBUG("New nodes names ID found!");
//                                 //   break;
//                                 // }
//       // check if param exist. if param does not exist but node is sill alive, the old node can be replaced by new one.
//       if (nh_.hasParam("/global_planner"+new_nodes_string_id+"/costmap/inflater/inflation_radius"))
//           new_nodes_ID++;
//   }

//   if (new_nodes_string_id =="empty"){

//     ROS_DEBUG("NO THREADS AVALIABLE FOR Path planner module!");
   
//   }else{
//       // set node names
//       ROS_DEBUG("setting nodes names");
//        std::string map_server_pkg_name = "map_server";
//        map_server_pkg_name = map_server_pkg_name + new_nodes_string_id;
//        std::string tf_broadcaster_pkg_name = "static_transform_publisher";
//        tf_broadcaster_pkg_name = tf_broadcaster_pkg_name + new_nodes_string_id;
//        std::string load_configs_pkg_name = "load_configs";
//        load_configs_pkg_name = load_configs_pkg_name + new_nodes_string_id;
//        std::string global_planner_pkg_name = "global_planner";
//        global_planner_pkg_name = global_planner_pkg_name + new_nodes_string_id;

//        pid_t map_server_pID = fork();
//        if (map_server_pID == 0)              
//        {
//           ROS_DEBUG("starting map_server");
          

//           execute_command = "/opt/ros/indigo/bin/rosrun";
//           execute_pkg_name = "map_server";
//           execute_pkg_exe_name = "map_server";
//           std:: string execute_param_1_string = "__name:="+map_server_pkg_name;
//           const char* execute_param_1 = execute_param_1_string.c_str();
//                     ROS_DEBUG_STREAM("map_server name:\n" << map_server_pkg_name );

//           // set map path
//           std:: string execute_param_2_string = map_path;
//           const char* execute_param_2 = execute_param_2_string.c_str();
//           // remap map publication topic
//           std:: string execute_param_3_string = "/map:=/"+map_server_pkg_name+"/map";
//           const char* execute_param_3 = execute_param_3_string.c_str();
//           execl(execute_command, execute_command,execute_pkg_name, execute_pkg_exe_name, execute_param_1, execute_param_2, execute_param_3, (char *)0);
//         }
//         else if (map_server_pID < 0)           
//         {
//             std::cout << "Failed to fork map_server" << std::endl;
//             exit(1);
//         }
//         else{
//            pid_t tf_broadcaster_pID = fork();
//            if (tf_broadcaster_pID == 0)               
//            {
//           ROS_DEBUG("starting tf_broadcaster_pID");
              
//               std::string tf_broadcaster_pkg_path = ros::package::getPath("tf");
//               execute_command = "/opt/ros/indigo/bin/rosrun";
//               execute_pkg_name = "tf";
//               execute_pkg_exe_name = "static_transform_publisher";
//               std:: string execute_param_1_string = "__name:="+tf_broadcaster_pkg_name;
//               const char* execute_param_1 = execute_param_1_string.c_str();
//               execl(execute_command,execute_command,execute_pkg_name, execute_pkg_exe_name, execute_param_1,  "0.066", "1.7", "0.54", "0.49999984", "0.49960184", "0.49999984","0.50039816","map","base_link","100", (char *)0);
//             }
//             else if (tf_broadcaster_pID < 0)          
//             {
//                 std::cout << "Failed to fork global_planner" << std::endl;
//                 exit(1);
//             }
//             else{
//                pid_t load_configs_costmap_pID = fork();
//                    if (load_configs_costmap_pID == 0)                
//                    {
//                       ROS_DEBUG("starting load_configs_costmap_pID");
                      
//                       execute_command = "/opt/ros/indigo/bin/rosparam";

//                       std::string costmap_file = "costmap_NAO.yaml";
//                       std::string load_configs_pkg_path = ros::package::getPath("rapp_path_planning");
//                       // set yaml file path
//                       std:: string execute_param_1_string = load_configs_pkg_path+"/cfg/costmap/"+costmap_file;
//                       const char* execute_param_1 = execute_param_1_string.c_str();
//                       ROS_DEBUG_STREAM("costmap file path:\n"<< execute_param_1_string);

//                       // set params namespace
//                       std:: string execute_param_2_string = "/"+global_planner_pkg_name+"/costmap/";
//                       const char* execute_param_2 = execute_param_2_string.c_str();

//                       execl(execute_command,execute_command,"load", execute_param_1,execute_param_2 , (char *)0);
//                     }
//                     else if (load_configs_costmap_pID < 0)         
//                     {
//                         std::cout << "Failed to fork load_configs" << std::endl;
//                         exit(1);
//                     }
//                     else{
//                             pid_t load_configs_planner_pID = fork();
//                          if (load_configs_planner_pID == 0)                
//                          {
//                             ROS_DEBUG("starting load_configs_planner_pID");

//                             execute_command = "/opt/ros/indigo/bin/rosparam";
                            
//                             std::string algorithm_file = "dijkstra.yaml";
//                             std::string load_configs_pkg_path = ros::package::getPath("rapp_path_planning");
//                             // set yaml file path
//                             std:: string execute_param_1_string =  load_configs_pkg_path+"/cfg/planner/"+algorithm_file;
//                             const char* execute_param_1 = execute_param_1_string.c_str();
//                             // set params namespace
//                             std:: string execute_param_2_string = "/"+global_planner_pkg_name+"/planner/";
//                             const char* execute_param_2 = execute_param_2_string.c_str();
//                             execl(execute_command,execute_command,"load", execute_param_1,execute_param_2 , (char *)0);
//                           }
//                           else if (load_configs_planner_pID < 0)            
//                           {
//                               ROS_ERROR("Failed to fork load_configs");
//                               exit(1);
                              
//                           }
//                           else{
//                                pid_t global_planner_pID = fork();
//                                  if (global_planner_pID == 0)                
//                                  {
//                                     ROS_DEBUG("starting global_planner_pID");

//                                     execute_command = "/opt/ros/indigo/bin/rosrun";

//                                     std::string global_planner_pkg_path = ros::package::getPath("global_planner");
//                                     execute_pkg_name = "global_planner";
//                                     execute_pkg_exe_name = "planner";
//                                     // ROS node name
//                                     std:: string execute_param_1_string = "__name:="+global_planner_pkg_name;
//                                     const char* execute_param_1 = execute_param_1_string.c_str();
//                                     // remap map subscribtion topic
//                                     std:: string execute_param_2_string =  "/map:=/"+map_server_pkg_name+"/map";
//                                     const char* execute_param_2 = execute_param_2_string.c_str();
//                                     std::string map_topic_test;
//                                     bool use_dijkstra_test;
//                                     while (!nh_.getParam("/"+global_planner_pkg_name+"/costmap/map_topic", map_topic_test) && !nh_.getParam("/"+global_planner_pkg_name+"/planner/use_dijkstra",use_dijkstra_test) ){
//                                     ROS_DEBUG("Global planner is waiting for configuration");
//                                     }
//                                     execl(execute_command,execute_command,execute_pkg_name, execute_pkg_exe_name, execute_param_1,  execute_param_2, (char *)0);

//                                   }
//                                   else if (global_planner_pID < 0)            
//                                   {
//                                        ROS_ERROR("Failed to fork global_planner");
//                                       exit(1);
                                      
//                                   }
//                                   else{
//                                       if(!nh_.getParam("/rapp_path_planning_plan_path_topic", pathPlanningTopic_))
//                                       {
//                                         ROS_ERROR("Path planning topic param does not exist");
//                                       }
//                                       ROS_DEBUG_STREAM("GP name:\n" << global_planner_pkg_name << "service name call:\n" << "/"+global_planner_pkg_name+"/make_plan");
//                                       uint32_t serv_port;
//                                       std::string serv_node_name, serv_name;
//                                       serv_name = "/"+global_planner_pkg_name+"/make_plan";
//                                       ros::ServiceManager srv_manager; 
//                                       bool serv_active = srv_manager.lookupService(serv_name,serv_node_name,serv_port);


//                                       while (!serv_active){
//                                         ROS_ERROR("Can't find service make_plan");
//                                       serv_active = srv_manager.lookupService(serv_name,serv_node_name,serv_port);
//                                       ros::Duration(1).sleep();
//                                       }
//                                        ROS_DEBUG_STREAM("HOST name:\n" << serv_node_name << "service port:\n" << serv_port);
                                       
//                                       ros::ServiceClient client = nh_.serviceClient<navfn::MakeNavPlan>("/"+global_planner_pkg_name+"/make_plan");
//                                       navfn::MakeNavPlan srv;
//                                       srv.request.start = request_start;
//                                       srv.request.goal = request_goal;
//                                       if (client.call(srv))
//                                       {
//                                             navfn::MakeNavPlanResponse planned_path; 
//                                             planned_path = srv.response;
//                                             ROS_DEBUG("Path planning service ended");
//                                             ROS_DEBUG_STREAM("\n nodes_ID:\n"<<"map_server: "<<map_server_pID <<"\ntf_broadcaster_pID: "<<tf_broadcaster_pID <<"\nload_configs_costmap_pID: "<<load_configs_costmap_pID <<"\nload_configs_planner_pID: "<<load_configs_planner_pID <<"\nglobal_planner_pID: "<<global_planner_pID);
//                                             pid_t killer_ID = fork();
//                                            if (killer_ID == 0)                
//                                            {
//                                             deleteParameters(new_nodes_string_id,nh_);
//                                              ROS_DEBUG("starting killer_ID");

//                                               execute_command = "/opt/ros/indigo/bin/rosnode";

//                                               // ROS node name
//                                               std:: string execute_param_1_string = global_planner_pkg_name;
//                                               const char* execute_param_1 = execute_param_1_string.c_str();
//                                               // remap map subscribtion topic
//                                               std:: string execute_param_2_string =  map_server_pkg_name;
//                                               const char* execute_param_2 = execute_param_2_string.c_str();
//                                               std:: string execute_param_3_string =  tf_broadcaster_pkg_name;
//                                               const char* execute_param_3 = execute_param_3_string.c_str();

//                                               execl(execute_command,execute_command,"kill", execute_param_1,  execute_param_2,execute_param_3, (char *)0);

//                                             }
//                                             else if (killer_ID < 0)            
//                                             {
//                                                 ROS_ERROR("Failed to fork global_planner");
//                                                 exit(1);
                                                
//                                             }
//                                             else{
//                                             return planned_path;

//                                           }
//                                       }
//                                       else
//                                       {
//                                         navfn::MakeNavPlanResponse planned_path; 
//                                         ROS_ERROR("Failed to call service make_plan");
//                                         kill(map_server_pID,SIGINT);
//                                         kill(tf_broadcaster_pID,SIGINT);
//                                         kill(load_configs_costmap_pID,SIGINT);
//                                         kill(load_configs_planner_pID,SIGINT);
//                                         kill(global_planner_pID,SIGINT);


//                                         return planned_path;
//                                       }
                  
//                                   }
                    
//                               }
//                           }
//                     }
            
//             }
            
//         }
// }

