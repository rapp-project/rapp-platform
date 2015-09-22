#include <face_detection/face_detection.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "face_detection_node");
  FaceDetection fdnode;

  ros::NodeHandle nh;
  int threads = 1;
  if(!nh.getParam("/rapp_face_detection_threads", threads))
  {
    ROS_ERROR("Face detection threads param not found");
  }
  else if(threads < 0)
  {
    threads = 1;
  }
  ros::MultiThreadedSpinner spinner(threads + 1); //Plus the standard one
  spinner.spin();
  return 0;
}
