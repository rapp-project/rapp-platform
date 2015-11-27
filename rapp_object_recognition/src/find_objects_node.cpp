#include "ros/ros.h"
#include "rapp_platform_ros_communications/FindObjectsSrv.h"

#include "rapp_object_recognition/find_objects.hpp"

bool service_FindObjects(rapp_platform_ros_communications::FindObjectsSrv::Request  &req,
                         rapp_platform_ros_communications::FindObjectsSrv::Response &res)
{
  FindObjects::findObjects(req.fname, req.names, req.files, req.limit, res.found_names, res.found_centers, res.found_scores);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_objects_node");
  ros::NodeHandle n;

  std::string service_name;
  if (!n.getParam("/rapp_object_recognition_topic", service_name))
    ROS_ERROR("rapp_object_recogntion_topic not set!");


  ros::ServiceServer service = n.advertiseService(service_name, service_FindObjects);
  ROS_INFO("Ready to find objects.");
  ros::spin();

  return 0;
}
