#include <path_planning/path_planning.h>

PathPlanning *path_planning_;

void mySigintHandler(int sig){
delete path_planning_;
ros::shutdown();
}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "path_planning_node");
    path_planning_ = new PathPlanning;

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
  signal(SIGINT, mySigintHandler); 
  spinner.spin();
  return 0;
}
