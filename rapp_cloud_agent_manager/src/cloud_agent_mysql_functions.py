#!/usr/bin/env python

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr
 
import rospy



from rapp_platform_ros_communications.srv import (
  fetchDataSrv,
  fetchDataSrvResponse,
  writeDataSrv,
  writeDataSrvResponse,
  deleteDataSrv,
  deleteDataSrvResponse,
  updateDataSrv,
  updateDataSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )


class CloudAgentMysqlFunctions:
  
  @staticmethod
  def writeCloudAgentEntry(user_id,image_identifier,tarball_path,container_identifier,container_type):
    serv_topic = rospy.get_param('rapp_mysql_wrapper_cloud_agent_write_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_cloud_agent_write_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    req = writeDataSrv()
    req.req_cols=["user_id","image_identifier","tarball_path","container_identifier","container_type"]    
    entry1=StringArrayMsg()
    entry1=[user_id,"'"+image_identifier+"'","'"+tarball_path+"'","'"+container_identifier+"'","'"+container_type+"'"]
    #entry1=[user_id,image_identifier,tarball_path,container_identifier,container_type]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.req_cols,req.req_data)
    print response.error
    return response.success.data

  @staticmethod
  def writeCloudAgentServiceEntry(cloud_agent_id,service_name,service_type,container_port,host_port):
    serv_topic = rospy.get_param('rapp_mysql_wrapper_cloud_agent_service_write_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_cloud_agent_service_write_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    req = writeDataSrv()
    req.req_cols=["cloud_agent_id","service_name","service_type","container_port","host_port"]    
    entry1=StringArrayMsg()
    entry1=[cloud_agent_id,"'"+service_name+"'","'"+service_type+"'",container_port,host_port]
    #entry1=[user_id,image_identifier,tarball_path,container_identifier,container_type]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.req_cols,req.req_data)
    print response.error
    return response.success.data

  @staticmethod
  def writeCloudAgentServiceArgumentsEntry(cloud_agent_service_id,argument_name):
    serv_topic = rospy.get_param('rapp_mysql_wrapper_cloud_agent_service_arguments_write_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_cloud_agent_service_arguments_write_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    req = writeDataSrv()
    req.req_cols=["cloud_agent_service_id","argument_name"]    
    entry1=StringArrayMsg()
    entry1=[cloud_agent_service_id,"'"+argument_name+"'"]
    #entry1=[user_id,image_identifier,tarball_path,container_identifier,container_type]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.req_cols,req.req_data)
    print response.error
    return response.success.data

