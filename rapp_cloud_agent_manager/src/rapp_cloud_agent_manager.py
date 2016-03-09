#!/usr/bin/env python
# -*- encode: utf-8 -*-

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
import sys

from create_container import CreateContainer


from rapp_platform_ros_communications.srv import (
  createContainerSrv,
  createContainerSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from std_msgs.msg import (
  String
  )


class RappCloudAgentManager:

  ## @brief Default contructor
  #
  # Waits for services the node depends on and declares the callbacks of the node's services
  def __init__(self):    
	  
	# Dependencies


    #Declare Callbacks
    self.serv_topic = rospy.get_param("rapp_cloud_agent_manager_create_container")
    if(not self.serv_topic):
      rospy.logerror("rapp_cloud_agent_manager_create_container")
    self.serv=rospy.Service(self.serv_topic, createContainerSrv, self.createContainerSrvDataHandler)  

    
  def createContainerSrvDataHandler(self,req):
    res = createContainerSrvResponse()
    it = CreateContainer()
    res=it.createContainer(req)
    return res
