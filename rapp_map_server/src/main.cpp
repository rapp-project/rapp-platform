/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */


/* modified by: Wojciech Dudek  

	Now map_server changes the map that is being published. Path to the map is subscibed from the node_name+"/setMap" parameter.
*/

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <sstream>
#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include "rapp_platform_ros_communications/MapServerGetMapRosSrv.h"
#include "rapp_platform_ros_communications/MapServerUploadMapRosSrv.h"
#include "std_srvs/Empty.h"


#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(std::string fname)
    {
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

      bool update_status = updateMap(fname,0);
      std::string node_name = ros::this_node::getName();

      get_service = n.advertiseService(node_name+"/get_map", &MapServer::mapCallback, this);
      upload_service = n.advertiseService(node_name+"/upload_map", &MapServer::mapUploadCallback, this);
      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      metadata_pub.publish( meta_data_message_ );
     
      // Latched publisher for data
      map_pub.publish( map_resp_.map );

    }

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer get_service,upload_service,test_service;
    std::string fname;
    bool deprecated;

    bool mapUploadCallback(rapp_platform_ros_communications::MapServerUploadMapRosSrv::Request  &req,
                     rapp_platform_ros_communications::MapServerUploadMapRosSrv::Response &res)
    {

    YAML::Node yaml_node;
    yaml_node["image"] = req.map_name+".png";
    yaml_node["resolution"] = req.resolution;
    yaml_node["origin"] = req.origin;
    yaml_node["negate"] = req.negate;
    yaml_node["occupied_thresh"] = req.occupied_thresh;


    std::ostringstream ss;
    yaml_node["free_thresh"] = req.free_thresh;

    std::string user = req.user_name;
    std::string map = req.map_name;
    std::string map_path ="/home/rapp/rapp_platform_files/maps/rapp/"+user;

    boost::filesystem::create_directories(map_path);
    std::string yaml_path = map_path+"/"+map+".yaml";
    std::ofstream fout(yaml_path.c_str());
    fout << yaml_node;
    std::string png_path = map_path+"/"+map+".png";

    std::ofstream outfile (png_path.c_str(),std::ofstream::binary);

    char* buffer = reinterpret_cast<char*>(req.data.data());
    long size = strlen(buffer);
    // write to outfile
    ROS_INFO_STREAM ("User: "<<user<< " saved map: "<< png_path << "\n size of the map: " << req.file_size << " bytes" );

    outfile.write (buffer,req.file_size);

      // // release dynamically-allocated memory
    outfile.close();
    res.status = true;
    return true;
    }
    /** Callback invoked when someone requests our service */
    bool mapCallback(rapp_platform_ros_communications::MapServerGetMapRosSrv::Request  &req,
                     rapp_platform_ros_communications::MapServerGetMapRosSrv::Response &res )
    {
      // request is empty; we ignore it
      std::string node_name = ros::this_node::getName();
      std::string param_name = node_name+"/setMap";

      const char * param_name_char = param_name.c_str();

      if (n.hasParam(param_name_char)){
         n.getParam(param_name_char,fname);
        updateMap(req.map_path,0);
      }
      res.map = map_resp_.map;
      return true;
    }
    bool updateMap(const std::string fname, double res){
      std::string mapfname = "";   
      double origin[3];
      int negate;
      double occ_th, free_th;
      bool trinary = true;
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("map"));
      deprecated = false;//(res != 0);
      if (!deprecated) {
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }
#ifdef HAVE_NEW_YAMLCPP
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try { 
          doc["resolution"] >> res; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["negate"] >> negate; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["occupied_thresh"] >> occ_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["free_thresh"] >> free_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["trinary"] >> trinary; 
        } catch (YAML::Exception) { 
          ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
          trinary = true;
        }
        try { 
          doc["origin"][0] >> origin[0]; 
          doc["origin"][1] >> origin[1]; 
          doc["origin"][2] >> origin[2]; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["image"] >> mapfname; 
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
      } else {
        private_nh.param("negate", negate, 0);
        private_nh.param("occupied_thresh", occ_th, 0.65);
        private_nh.param("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, trinary);

      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;
      map_pub.publish( map_resp_.map );

      return true;
    }
    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server");
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  if (argc != 2) {
    ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  double res = (argc == 2) ? 0.0 : atof(argv[2]);

  try
  {    
	MapServer ms(fname);

    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}


