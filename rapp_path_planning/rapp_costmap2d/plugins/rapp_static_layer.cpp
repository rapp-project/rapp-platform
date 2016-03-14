/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <rapp_costmap_2d/rapp_static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::RappStaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{
RappStaticLayer::RappStaticLayer() : dsrv_(NULL) {}

RappStaticLayer::~RappStaticLayer()
{
    if(dsrv_)
        delete dsrv_;
}

void RappStaticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_service;
  nh.param("map_service", map_service, std::string("/static_map"));
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);
  
  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("Requesting the map...");

  ros::ServiceManager srv_manager;
std::string host_name;
uint32_t port_srv;
    ROS_DEBUG_STREAM("COSTMAP calls:\n" << map_service);

  if (srv_manager.lookupService(map_service,host_name, port_srv)){
  ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>(map_service);
  nav_msgs::GetMap srv;

  if (map_client.call(srv))
  { 
   nav_msgs::OccupancyGrid const new_map = srv.response.map;

auto a_ptr=boost::make_shared<::nav_msgs::OccupancyGrid const>(new_map);
  // nav_msgs::OccupancyGrid *new_map = new res.map;
  // boost::shared_ptr<nav_msgs::OccupancyGrid> new_map_ptr(new_map);
   nav_msgs::OccupancyGridConstPtr const new_map_occupancy_ptr = a_ptr;
        RappStaticLayer::incomingMap(new_map_occupancy_ptr);
        // ROS_INFO("Costmap2D got new map");
        //    std::string node_name = ros::this_node::getName();
        //        ros::ServiceServer costmap_update_service = g_nh.advertiseService(node_name+"/costmap_map_update", &RappStaticLayer::incomingUpdateService, this);

  }
  
  else
  {
    ROS_ERROR("Costmap2D can not request new map from rapp_map_server");
    ROS_ERROR_STREAM("Requested service:\n"<<map_service);
  }

  }
  //map_sub_ = g_nh.subscribe(map_service, 1, &RappStaticLayer::incomingMap, this);
  map_received_ = true;
  has_updated_data_ = true;

  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
    std::string node_name = ros::this_node::getName();
   // std::cout << "NODENAME= " << node_name <<std::endl;
  ROS_INFO_STREAM("Starting rapp_costmap for: " << node_name);

  ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
  
//  if(subscribe_to_updates_)
//  {
   // std::string node_name = ros::this_node::getName();
    ROS_DEBUG("Subscribing to updates");
    costmap_update_service = g_nh.advertiseService(node_name+"/costmap_map_update", &RappStaticLayer::incomingUpdateService, this);

    //map_update_sub_ = g_nh.advertizeService("/"+name_ + "_costmap_update", 10, &RappStaticLayer::incomingUpdate, this);
 // }

  if(dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &RappStaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
bool RappStaticLayer::incomingUpdateService(  rapp_platform_ros_communications::Costmap2dRosSrvRequest& req,
  rapp_platform_ros_communications::Costmap2dRosSrvResponse& res){

   nav_msgs::OccupancyGrid const new_map = req.map;

auto a_ptr=boost::make_shared<::nav_msgs::OccupancyGrid const>(new_map);
  // nav_msgs::OccupancyGrid *new_map = new res.map;
  // boost::shared_ptr<nav_msgs::OccupancyGrid> new_map_ptr(new_map);
   nav_msgs::OccupancyGridConstPtr const new_map_occupancy_ptr = a_ptr;
        RappStaticLayer::incomingMap(new_map_occupancy_ptr);
        res.status = true;
        return true;
}
void RappStaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void RappStaticLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

unsigned char RappStaticLayer::interpretValue(unsigned char value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void RappStaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked())
  {
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }else if(size_x_ != size_x || size_y_ != size_y ||
      resolution_ != new_map->info.resolution ||
      origin_x_ != new_map->info.origin.position.x ||
      origin_y_ != new_map->info.origin.position.y){
    matchSize();
  }

  unsigned int index = 0;

  //initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}

void RappStaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
    unsigned int di = 0;
    for (unsigned int y = 0; y < update->height ; y++)
    {
        unsigned int index_base = (update->y + y) * size_x_;
        for (unsigned int x = 0; x < update->width ; x++)
        {
            unsigned int index = index_base + x + update->x;
            costmap_[index] = interpretValue( update->data[di++] );
        }
    }
    x_ = update->x;
    y_ = update->y;
    width_ = update->width;
    height_ = update->height;
    has_updated_data_ = true;
}

void RappStaticLayer::activate()
{
    onInitialize();
}

void RappStaticLayer::deactivate()
{
    map_sub_.shutdown();
    if (subscribe_to_updates_)
        map_update_sub_.shutdown();
}

void RappStaticLayer::reset()
{
    deactivate();
    activate();
}

void RappStaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
    return;
    
  useExtraBounds(min_x, min_y, max_x, max_y);

  double mx, my;
  
  mapToWorld(x_, y_, mx, my);
  *min_x = std::min(mx, *min_x);
  *min_y = std::min(my, *min_y);
  
  mapToWorld(x_ + width_, y_ + height_, mx, my);
  *max_x = std::max(mx, *max_x);
  *max_y = std::max(my, *max_y);
  
  has_updated_data_ = false;
}

void RappStaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;
  if (!use_maximum_)
    updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
  else
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace rapp_costmap_2d
