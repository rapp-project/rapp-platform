#include <face_detection_wrapper/face_detection_wrapper_class.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "face_detection_wrapper_node");
  FaceDetectionWrapper fdnode;
  ros::spin();
  return 0;
}
