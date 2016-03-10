#!/usr/bin/env python2
# -*- coding: utf-8 -*-

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

import rospy
import json
import sys
import os

from rapp_platform_ros_communications.srv import (
  ApplicationTokenGenerationSrv,
  ApplicationTokenGenerationSrvResponse
  )

from rapp_exceptions import RappError

class ApplicationAuthenticationManager:

  def __init__(self):
    self.generate_token_topic = rospy.get_param("rapp_generate_token_topic")
    if(not self.generate_token_topic):
        rospy.logerr("Application authentication: Token generation topic does not exist")

    self.generate_token_service = rospy.Service(self.generate_token_topic, \
        ApplicationTokenGenerationSrv, self.generate_token_callback)

  # The service callback
  def generate_token_callback(self, req):

    res = ApplicationTokenGenerationSrvResponse()
    res.error = ''
    res.token = 'test'
    return res

if __name__ == "__main__":
  rospy.init_node('application_authentication_node')
  application_authentication_ros_node = ApplicationAuthenticationManager()
  rospy.spin()


