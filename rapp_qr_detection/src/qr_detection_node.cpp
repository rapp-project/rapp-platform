#include <qr_detection/qr_detection.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "qr_detection_node");
  QrDetection qrnode;

  ros::NodeHandle nh;
  int threads = 1;
  if(!nh.getParam("/rapp_qr_detection_threads", threads))
  {
    ROS_ERROR("Qr detection threads param not found");
  }
  else if(threads < 0)
  {
    threads = 1;
  }
  ros::MultiThreadedSpinner spinner(threads); //Plus the standard one
  spinner.spin();
  return 0;
}
