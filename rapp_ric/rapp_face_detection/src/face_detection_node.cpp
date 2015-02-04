#include <face_detection/face_detection.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "face_detection_node");
  FaceDetection fdnode;
  ros::spin();
  return 0;
}
