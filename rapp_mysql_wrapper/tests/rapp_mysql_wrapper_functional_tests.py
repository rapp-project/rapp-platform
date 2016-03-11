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

PKG='test_rapp_mysql_wrapper'
import sys
import unittest
import rospy

from rapp_platform_ros_communications.srv import (
  authTokenByServiceSrv,
  authTokenByServiceSrvResponse,
  getUserOntologyAliasSrv,
  getUserOntologyAliasSrvResponse,
  getUserOntologyAliasSrvRequest,
  registerUserOntologyAliasSrv,
  registerUserOntologyAliasSrvResponse,
  getUserLanguageSrv,
  getUserLanguageSrvResponse,
  registerNewTokenSrv,
  registerNewTokenSrvResponse,
  registerNewTokenServiceSrv,
  registerNewTokenServiceSrvResponse,
  getServicesByTokenSrv,
  getServicesByTokenSrvResponse
  )

## @class TestDbWrapper 
# Inherits the unittest.TestCase class in order to offer functional tests functionality 
class TestDbWrapper(unittest.TestCase):
  ## Tests the rapp_mysql_wrapper_user_write_data service when an invalid column is provided 
  def testgetUserOntologyAlias(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_get_user_ontology_alias_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_get_user_ontology_alias_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, getUserOntologyAliasSrv)
    req = getUserOntologyAliasSrvRequest()
    req.user_id="0"        
    response = test_service(req)     
    self.assertEqual(response.success, True) 
    self.assertEqual(response.ontology_alias,"Person_DpphmPqg")

## The main function. Initializes the Rapp mysql wrapper functional tests
if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)    
