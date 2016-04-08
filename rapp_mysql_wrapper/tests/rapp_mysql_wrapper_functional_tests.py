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
  getUserOntologyAliasSrv,
  getUserOntologyAliasSrvRequest,
  getUserOntologyAliasSrvResponse,
  registerUserOntologyAliasSrv,
  registerUserOntologyAliasSrvRequest,
  registerUserOntologyAliasSrvResponse,
  checkIfUserExistsSrv,
  checkIfUserExistsSrvResponse,
  checkIfUserExistsSrvRequest,
  getUserLanguageSrv,
  getUserLanguageSrvRequest,
  getUserLanguageSrvResponse,
  getUserPasswordSrv,
  getUserPasswordSrvResponse,
  getUserPasswordSrvRequest,
  getUsernameAssociatedWithApplicationTokenSrv,
  getUsernameAssociatedWithApplicationTokenSrvResponse,
  getUsernameAssociatedWithApplicationTokenSrvRequest,
  createNewPlatformUserSrv,
  createNewPlatformUserSrvResponse,
  createNewPlatformUserSrvRequest,
  createNewApplicationTokenSrv,
  createNewApplicationTokenSrvRequest,
  createNewApplicationTokenSrvResponse,
  checkActiveApplicationTokenSrv,
  checkActiveApplicationTokenSrvRequest,
  checkActiveApplicationTokenSrvResponse,
  removePlatformUserSrv,
  removePlatformUserSrvRequest,
  removePlatformUserSrvResponse
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
    req.username="rapp"        
    response = test_service(req)     
    self.assertTrue(response.success) 
    self.assertEqual(response.ontology_alias,"Person_DpphmPqg")
    
  def testRegisterUserOntologyAlias(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_register_user_ontology_alias_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_register_user_ontology_alias_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, registerUserOntologyAliasSrv)
    req = registerUserOntologyAliasSrvRequest()
    req.username="rapp"
    req.ontology_alias="Person_DpphmPqg"        
    response = test_service(req)     
    self.assertTrue(response.success) 
    
  def testCheckIfUserExists(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_check_if_user_exists_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_check_if_user_exists_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, checkIfUserExistsSrv)
    req = checkIfUserExistsSrvRequest()
    req.username="rapp"       
    response = test_service(req)     
    self.assertTrue(response.success) 
    self.assertTrue(response.user_exists)  
    test_service = rospy.ServiceProxy(ros_service, checkIfUserExistsSrv)
    req = checkIfUserExistsSrvRequest()
    req.username="rapp23243"       
    response = test_service(req)     
    self.assertTrue(response.success) 
    self.assertTrue(not response.user_exists) 

    
  def testgetUserLanguage(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_get_user_language_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_get_user_language_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, getUserLanguageSrv)
    req = getUserLanguageSrvRequest()
    req.username="rapp"        
    response = test_service(req)     
    self.assertTrue(response.success) 
    self.assertEqual(response.user_language,"el")

  def testgetUserPassword(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_get_user_password_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_get_user_password_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, getUserPasswordSrv)
    req = getUserPasswordSrvRequest()
    req.username="rapp"        
    response = test_service(req)     
    self.assertTrue(response.success) 
    self.assertEqual(response.password,"$2b$12$0RzTZr6bjbqRDTzT4SYBV.I44fG6RHUjMtqxeP2c6Qaansh03GhTC")
    
  def testgetUsernameAssociatedWithApplicationToken(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_get_username_associated_with_application_token_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_get_username_associated_with_application_token_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, getUsernameAssociatedWithApplicationTokenSrv)
    req = getUsernameAssociatedWithApplicationTokenSrvRequest()
    req.application_token="rapp_token"        
    response = test_service(req)     
    self.assertTrue(response.success) 
    self.assertEqual(response.username,"rapp")    
 
    
  def testcheckActiveApplicationToken(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_check_active_application_token_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_check_active_application_token_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, checkActiveApplicationTokenSrv)
    req = checkActiveApplicationTokenSrvRequest()
    req.application_token="rapp_token"        
    response = test_service(req)     
    self.assertTrue(response.success) 
    self.assertTrue(response.application_token_exists) 
    
  def testCreateAndRemoveNewPlatformUser(self):
    ros_service = rospy.get_param("rapp_mysql_wrapper_create_new_platform_user_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_create_new_platform_user_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, createNewPlatformUserSrv)
    req = createNewPlatformUserSrvRequest()
    req.username="temp_to_delete"
    req.password="temp_to_delete"
    req.language="el"
    req.creator_username="rapp"              
    response = test_service(req)     
    #self.assertTrue(response.success) 
    self.assertEqual(response.error,"")
    
    ros_service = rospy.get_param("rapp_mysql_wrapper_remove_platform_user_service_topic")
    if(not ros_service):
      rospy.logerror("rapp_mysql_wrapper_remove_platform_user_service_topic NOT FOUND ERROR")
    rospy.wait_for_service(ros_service)    
    test_service = rospy.ServiceProxy(ros_service, removePlatformUserSrv)
    req = removePlatformUserSrvRequest()
    req.username="temp_to_delete"             
    response = test_service(req)     
    self.assertTrue(response.success) 


## The main function. Initializes the Rapp mysql wrapper functional tests
if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)    
