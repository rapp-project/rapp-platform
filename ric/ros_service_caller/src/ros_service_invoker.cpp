#include <ros_service_invoker/ros_service_factory.hpp>

int main (void){

  faceDetectorCaller* t = 
    RosInvokerFactory::getInvoker<int, FaceDetectionStrategies>(FACE_DETECTION); 

  t->set_strategy(INT_2);
  t->setup(4);

  return 0;
}
