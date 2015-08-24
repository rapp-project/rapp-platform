#include <face_detection/face_detection.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "face_detection_node");
  FaceDetection fdnode;
  ros::MultiThreadedSpinner spinner(10);
  spinner.spin();
  return 0;
}
