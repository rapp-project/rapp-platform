#ifndef RAPP_FACE_DETECTION_WRAPPER
#define RAPP_FACE_DETECTION_WRAPPER

#include <iostream>
#include "ros/ros.h"

#include <rapp_platform_ros_communications/FaceDetectionHOPWrapMsg.h>
#include <rapp_platform_ros_communications/FaceDetectionWrapHOPMsg.h>

class FaceDetectionWrapper 
{
  public:

    // Default constructor
    FaceDetectionWrapper (void);

    void hop2wrapperCallback(const 
      rapp_platform_ros_communications::FaceDetectionHOPWrapMsg& msg);
    
  private:
    // The ROS ndoe handle
    ros::NodeHandle nh_;

    // Publishers for communicating with the HOP service. Notice that both
    // publishers must be initiated from here. Hop can declare its own 
    // publisher?
    ros::Publisher hop2wrapperPublisher_;
    ros::Publisher wrapper2hopPublisher_;
    
    // Subscriber that listens to to hop2wrapper topic
    ros::Subscriber hop2wrapperSubscriber_;

    // Topic nomeclarure
    std::string hop2wrapperTopic_;
    std::string wrapper2hopTopic_;

    
};

#endif
