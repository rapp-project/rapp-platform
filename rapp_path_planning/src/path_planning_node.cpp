#include <path_planning/path_planning.h>



int main(int argc, char ** argv)
{

  ROS_INFO("std::cout << before ROS init");
  ros::init(argc, argv, "path_planning_node");
 ROS_INFO("before class init");

  PathPlanning ppnode;
 ROS_INFO("after class init");

  ros::NodeHandle nh;
  int threads = 1;
  if(!nh.getParam("/rapp_path_planning_threads", threads))
  {
    ROS_ERROR("Path planning threads param does not exist. Setting 5 threads.");
    threads = 5;
  }
  else if(threads < 0)
  {
    threads = 1;
  }
  ros::MultiThreadedSpinner spinner(threads); 
  spinner.spin();
  return 0;
}
