#include <rapp_map_builder_2d/map_builder_2d.h>
#include <signal.h>


MapBuilder2d *MapBuilder2d_;

void mySigintHandler(int sig){
delete MapBuilder2d_;
ros::shutdown();
}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "map_builder_2d_node");
  ros::NodeHandle nh;
  int threads = 1;
  if(!nh.getParam("/rapp_build_map_2d_threads", threads))
  {
    ROS_WARN("Map builder 2d threads param does not exist. Setting 5 threads.");
    threads = 5;
  }
  else if(threads < 0)
  {
    threads = 1;
  }

  MapBuilder2d_ = new MapBuilder2d(threads);

  ros::MultiThreadedSpinner spinner(threads);
  signal(SIGINT, mySigintHandler); 
  spinner.spin();
  return 0;
}
