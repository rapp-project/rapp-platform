#include <path_planning/path_planning.h>



int main(int argc, char ** argv)
{

  ros::init(argc, argv, "path_planning_node");

  PathPlanning ppnode;

  ros::NodeHandle nh;
  int threads = 1;
  if(!nh.getParam("/rapp_path_planning_threads", threads))
  {
    ROS_WARN("Path planning threads param does not exist. Setting 5 threads.");
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
