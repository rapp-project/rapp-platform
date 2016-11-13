
#include <sstream>

#include <iostream>
#include <string>

#include <rapp_map_builder_2d/map_builder_2d.h>
#include <ros/package.h>

MapBuilder2d::MapBuilder2d(int req_thread_number){
      IDs.clear();
      thread_number = req_thread_number;
      for(int i=0; i<thread_number;i++){
        IDs.push_back(false);
      }

  if(!n_.getParam("rapp_map_builder_2d_topic", srv_name_))
  {
    ROS_WARN("Map builder 2d topic param does not exist. Setting srv_name = /rapp/rapp_map_builder_2d/build_map_2d");
    srv_name_ = "/rapp/rapp_map_builder_2d/build_map_2d";
  }
 map_building_service = n_.advertiseService(srv_name_, &MapBuilder2d::build_map_2d, this);
}

MapBuilder2d::~MapBuilder2d(){

}

bool MapBuilder2d::GetID(std::string &ID){
  int i = 0;
  pthread_mutex_lock(&getID_lock_);
  for (i=0; i<thread_number;i++){
    if (IDs.at(i) == false){
      IDs.at(i) = true;
      break;
    } 
  }
  pthread_mutex_unlock(&getID_lock_);
  std::ostringstream ss;
  ss.str("");
  ss << i; 
  ID = ss.str();   
}

bool MapBuilder2d::FreeID(std::string &ID){
  int i = 0;
  std::istringstream iss(ID);
  iss >> i;
  
  pthread_mutex_lock(&getID_lock_);
  IDs.at(i) = false;
  pthread_mutex_unlock(&getID_lock_);
}

bool MapBuilder2d::save_map(std::string &ID, std::string &map_path){

  std::string map_start = "rosrun map_server map_saver -f "+map_path+" map:=map_"+ID;
  system(map_start.c_str());

}
bool MapBuilder2d::build_map_2d(rapp_platform_ros_communications::BuildMap2DRosSrv::Request  &req,
                  rapp_platform_ros_communications::BuildMap2DRosSrv::Response &res){
  std::string map_path = req.map_path;
  std::string bag_file_path = req.bag_file_path;
  std::string my_ID = "";

  GetID(my_ID);
  std::stringstream ss_save_map;

  std::string my_pkg_path = ros::package::getPath("rapp_map_builder_2d");
  ss_save_map << "' "+my_pkg_path+"/../../../devel/lib/rapp_map_builder_2d/map_saver "<<my_ID<<" "<<map_path<<"'";
  std::stringstream ss_build_map;

           ss_build_map << "rosrun gmapping slam_gmapping_replay --bag_filename "+bag_file_path+" --on_done "<<ss_save_map.str()<<" --scan_topic /base_scan map:=map_"<<my_ID<<" tf:=/tf_"<<my_ID<<" scan:=/base_scan_"<<my_ID<<" __name:=gmapping_"+my_ID;
        //std::cout<<all_start<<std::endl;
        std::cout<<ss_build_map.str()<<std::endl;
        system(ss_build_map.str().c_str()); 

  FreeID(my_ID);
  std::string kill = "rosnode kill gmapping_"+my_ID+" map_server_"+my_ID+" ";
  system(kill.c_str());
}

// bool map_status = false;
// pthread_mutex_t lock;



// void start_gmapping(std::string node_name){

//  std::string gmapping_start = "rosrun gmapping slam_gmapping scan:=base_scan_"+node_name+" clock:=clock_"+node_name+" tf:=tf_"+node_name+" tf_static:=tf_static_"+node_name+" map:=map_"+node_name+" map_metadata:=map_metadata_"+node_name;
//  system(gmapping_start.c_str());
// }

// void start_rosbag(std::string node_name,  ros::NodeHandle *n){
//     ros::Publisher laser_pub = n->advertise<sensor_msgs::LaserScan>("base_scan_"+node_name, 100);
//     ros::Publisher tf_pub = n->advertise<tf::tfMessage>("tf_"+node_name, 100);
//     ros::Publisher tf_static_pub = n->advertise<tf2_msgs::TFMessage>("tf_static_"+node_name, 100);
//     ros::Publisher clock_pub = n->advertise<rosgraph_msgs::Clock>("clock", 100);
//     int test;
//     n->getParam("test_param", test );
//     std::cout<<"test_param: "<<test<<std::endl;
//     std::string bagFile_path = "/home/wojciech/Downloads/basic_localization_stage.bag";

//     rosbag::Bag bag;
//     bag.open(bagFile_path, rosbag::bagmode::Read);

//     std::string laser_topic = "/base_scan";
//     std::string tf = "/tf";
//     std::string tf_static = "/tf_static";
//     std::string clock = "/clock";

//     std::vector<std::string> topics;
//     topics.push_back(laser_topic);
//     topics.push_back(tf);
//     topics.push_back(tf_static);
//     topics.push_back(clock);

//     rosbag::View view(bag, rosbag::TopicQuery(topics));
//     std::cout<<"rosbag--view"<<std::endl;
//     BOOST_FOREACH(rosbag::MessageInstance const m, view)
//     {
//         std::cout<<"rosbag--laser_1"<<std::endl;

//       if (m.getTopic() == laser_topic) 
//       {
//         std::cout<<"rosbag--laser_2"<<std::endl;

//         sensor_msgs::LaserScan::ConstPtr laser_ptr = m.instantiate<sensor_msgs::LaserScan>();
//         if (laser_ptr != NULL){
//             std::cout<<"rosbag--laser_3"<<std::endl;

//           laser_pub.publish(*laser_ptr);
//         }
//       }

//       if (m.getTopic() == tf) 
//       {
//             std::cout<<"rosbag--tf_2"<<std::endl;

//         tf::tfMessage::ConstPtr tf_ptr = m.instantiate<tf::tfMessage>();
//         if (tf_ptr != NULL){
//                     std::cout<<"rosbag--tf_3"<<std::endl;

//           tf_pub.publish(*tf_ptr);
//         }
//       }

//       if (m.getTopic() == tf_static) 
//       {
//         tf2_msgs::TFMessage::ConstPtr tf_static_ptr = m.instantiate<tf2_msgs::TFMessage>();
//         if (tf_static_ptr != NULL)
//           tf_static_pub.publish(*tf_static_ptr);
//       }
//    if (m.getTopic() == clock) 
//       {
//         rosgraph_msgs::Clock::ConstPtr clock_ptr = m.instantiate<rosgraph_msgs::Clock>();
//         if (clock_ptr != NULL){
//           clock_pub.publish(*clock_ptr);
//         }
//       }

//     }

//   bag.close();
//   pthread_mutex_lock(&lock);

//   map_status =true;
//   pthread_mutex_unlock(&lock);
//  }

// // void *start_map_saver(void *){

// //  std::string gmapping_start = "rosrun gmapping slam_gmapping scan:=base_scan_"+node_name+"3 tf:=tf_"+node_name+" tf_static:=tf_static_"+node_name+" map:=map_"+node_name+" map_metadata:=map_metadata_"+node_name;
// //  system(gmapping_start.c_str());
// //     return NULL;
// // }


int test(int argc, char** argv)
{
     ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    std::cout<<"test"<<std::endl;
 std::string node_name = ros::this_node::getName();
 std::string map_name = "koziol";
 //char** = 
/*
std::thread t;
std::thread t1;
 pthread_t gmapping;
 pthread_t rosbag;
 pthread_t map_saver;
std::cout<<"test"<<std::endl;
 ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    std::cout<<"test"<<std::endl;
 std::string node_name = ros::this_node::getName();
    
    std::cout<<"test"<<std::endl;
    t=std::thread(start_gmapping, node_name);
    std::cout<<"test"<<std::endl;
    t1=std::thread(start_rosbag, node_name, &n);
    std::cout<<"test"<<std::endl;
    //pthread_create(&rosbag, NULL, start_rosbag, node_name);
    while (ros::ok()){
         pthread_mutex_lock(&lock);
         if (map_status)
             break;
         pthread_mutex_unlock(&lock);
      sleep(1);   
     }
    //std::string rosbag_start = "rosbag play --clock ~/Downloads/basic_localization_stage.bag base_scan:=base_scan_ tf:=tf_";
    //        system(rosbag_start.c_str());

        std::string map_start = "rosrun map_server map_saver -f my_new_map map:=map_"+node_name;
        system(map_start.c_str());
    
         //Join the thread with the main thread
     //    pthread_join(t, NULL);
         t1.join();
         t.join();
         --on_done 'rosrun map_server map_saver -f haha map:=map_"+node_name+" '
*/
 std::stringstream ss;
ss << "' rosrun 2d_map_builder map_saver "<<node_name<<" "<<map_name<<"'";
                 std::string map_start = "rosrun map_server map_saver map:=map_"+node_name+" -f my_new_map ";
 std::stringstream sss;

           sss << "rosrun gmapping slam_gmapping_replay --bag_filename /home/wojciech/Downloads/basic_localization_stage.bag  --on_done "<<ss.str()<<" --scan_topic /base_scan map:=map_"<<node_name<<" tf:=/tf_"<<node_name<<" scan:=/base_scan_"<<node_name;
        //std::cout<<all_start<<std::endl;
        std::cout<<sss.str()<<std::endl;
        system(sss.str().c_str()); 
        //         std::string map_start = "rosrun map_server map_saver -f my_new_map map:=map_"+node_name;
        // system(map_start.c_str());
 return(0);
   
}