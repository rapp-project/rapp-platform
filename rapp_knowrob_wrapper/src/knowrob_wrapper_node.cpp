#include <knowrob_wrapper/knowrob_wrapper_communications.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "knowrob_wrapper_node");
  KnowrobWrapperCommunications krcnode;
  ros::MultiThreadedSpinner spinner(10);
  spinner.spin();
  return 0;
}
