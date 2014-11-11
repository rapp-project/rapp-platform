#include <face_detection_wrapper/face_detection_wrapper_class.h>

FaceDetectionWrapper::FaceDetectionWrapper(void)
{
  hop2wrapperTopic_ = std::string("face_detection_h2r");
  wrapper2hopTopic_ = std::string("face_detection_r2h");

  hop2wrapperPublisher_ = nh_.advertise
    <rapp_platform_ros_communications::FaceDetectionHOPWrapMsg>(
      hop2wrapperTopic_, 1000);

  wrapper2hopPublisher_ = nh_.advertise
    <rapp_platform_ros_communications::FaceDetectionWrapHOPMsg>(
      wrapper2hopTopic_, 1000);

  hop2wrapperSubscriber_ = nh_.subscribe(hop2wrapperTopic_, 1,
    &FaceDetectionWrapper::hop2wrapperCallback, this);
}

void FaceDetectionWrapper::hop2wrapperCallback(
  const rapp_platform_ros_communications::FaceDetectionHOPWrapMsg& msg)
{
  std::cout << "I'm here!\n";
}


