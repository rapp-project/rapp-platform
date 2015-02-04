#include <qr_detection/qr_detection.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "qr_detection_node");
  QrDetection qrnode;
  ros::spin();
  return 0;
}
