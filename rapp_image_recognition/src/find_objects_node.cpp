#include "ros/ros.h"
#include "rapp_image_recognition/FindObjects.h"

#include "rapp_image_recognition/find_objects.hpp"

bool service_FindObjects(rapp_image_recognition::FindObjects::Request  &req,
                         rapp_image_recognition::FindObjects::Response &res)
{
  FindObjects::findObjects(req.fname, req.names, req.files, req.limit, res.found_names, res.found_centers, res.found_scores);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_objects_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("find_objects", service_FindObjects);
  ROS_INFO("Ready to find objects.");
  ros::spin();

  return 0;
}
