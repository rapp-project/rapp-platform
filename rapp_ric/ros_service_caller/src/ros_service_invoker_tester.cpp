#include <ros_service_invoker/ros_service_factory.hpp>

// Tester for the face detection invoker

int main (int argc, char** argv){

  // Typical ROS initializations
  ros::init(argc, argv, "ros_service_invoker_tester_node");
  std::string auxiliary_package = ros::package::getPath("rapp_auxiliary_files"); 
  ros::spinOnce();

  //------------------------- The invoker code ------------------------------//
  #define SETUP std::string, FaceDetectionStrategies
  // Getting the face detection invoker using string as setup and declaring the
  // enum containing the strategies
  IRosServiceInvoker<SETUP>* t = 
    RosInvokerFactory::getInvoker<SETUP>(FACE_DETECTION); 
 
  // Selecting the image URL strategy
  t->set_strategy(FaceDetectionStrategies::STRING_IMAGE_URL);
 
  // Sets up the service message
  t->setup(auxiliary_package + std::string("/Lenna.png"));
 
  // Calls the message and gets the response
  // TODO: Template the response as well
  std::string response = t->call_service();
 
  // Deletes the invoker
  delete t;
  //-------------------------------------------------------------------------//
  
  std::cout << "Face detection responded :" << response << "\n";

  return 0;
}
