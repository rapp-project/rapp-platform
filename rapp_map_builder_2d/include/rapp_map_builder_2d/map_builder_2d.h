#ifndef RAPP_2D_MAP_BUILDER_NODE
#define RAPP_2D_MAP_BUILDER_NODE

#include <string>
#include <rapp_platform_ros_communications/BuildMap2DRosSrv.h>
#include <vector>
#include <pthread.h>
#include <ros/ros.h>

class MapBuilder2d
{
  public:

	MapBuilder2d(int req_thread_number);
	~MapBuilder2d(void);

	bool save_map(std::string &ID, std::string &map_path);

	bool build_map_2d(rapp_platform_ros_communications::BuildMap2DRosSrv::Request &req,
                  rapp_platform_ros_communications::BuildMap2DRosSrv::Response &res);
  private:
	ros::NodeHandle n_;
	
	bool GetID(std::string &ID);
bool FreeID(std::string &ID);
 std::string srv_name_;

	int thread_number;
  	std::vector<bool> IDs;
	pthread_mutex_t getID_lock_;
	ros::ServiceServer map_building_service;
};


#endif