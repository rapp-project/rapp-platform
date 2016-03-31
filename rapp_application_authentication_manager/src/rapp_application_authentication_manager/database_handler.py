#!/usr/bin/env python
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

# Authors: Aris Thallas, Athanassios Kintsakis
# contact: aris.thallas@{iti.gr, gmail.com}

from rapp_platform_ros_communications.srv import (
  getUserOntologyAliasSrv,
  getUserOntologyAliasSrvResponse,
  getUserOntologyAliasSrvRequest,
  registerUserOntologyAliasSrv,
  registerUserOntologyAliasSrvResponse,
  registerUserOntologyAliasSrvRequest,
  checkIfUserExistsSrv,
  checkIfUserExistsSrvResponse,
  checkIfUserExistsSrvRequest,
  getUserLanguageSrv,
  getUserLanguageSrvResponse,
  getUserLanguageSrvRequest,
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
  checkActiveApplicationTokenSrvResponse
}

class DatabaseHandler(object):

    def __init__(self):
        #  TODO: initialize sql wrapper
        pass

    ## Verify that the token exists in store db
    #
    # @param store_token [string] The token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_store_token(self, store_token):
        #  TODO
        pass

    ## Verify that username exists in platform db
    #
    # @param username [string] The username
    #
    # @return status [bool] True if username exists, false otherwise
    def username_exists(self, username):
      ros_service = rospy.get_param("rapp_mysql_wrapper_check_if_user_exists_service_topic")
      if(not ros_service):
        rospy.logerror("rapp_mysql_wrapper_check_if_user_exists_service_topic NOT FOUND ERROR")
      rospy.wait_for_service(ros_service)    
      test_service = rospy.ServiceProxy(ros_service, checkIfUserExistsSrv)
      req = checkIfUserExistsSrvRequest()
      req.username=username     
      response = test_service(req)     
      if(response.success):
        return response.user_exists
      else
        return false

    ## Retrieve user's password from platform db
    #
    # @param username [string] The username
    #
    # @return password [string] Password associated with username
    def get_user_password(self, username):
      ros_service = rospy.get_param("rapp_mysql_wrapper_get_user_password_service_topic")
      if(not ros_service):
        rospy.logerror("rapp_mysql_wrapper_get_user_password_service_topic NOT FOUND ERROR")
      rospy.wait_for_service(ros_service)    
      test_service = rospy.ServiceProxy(ros_service, getUserPasswordSrv)
      req = getUserPasswordSrvRequest()
      req.username=username        
      response = test_service(req)     
      if(response.success):
        return response.password
      else:
        #raise error


    #def verify_active_application_token(self, token):
        #  TODO
     #   pass

    ## Retrieve username associated with application_token from platform db
    #
    # @param token [string] The user's application token
    #
    # @return username [string] The username
    def get_token_user(self, app_token):
      ros_service = rospy.get_param("rapp_mysql_wrapper_get_username_associated_with_application_token_service_topic")
      if(not ros_service):
        rospy.logerror("rapp_mysql_wrapper_get_username_associated_with_application_token_service_topic NOT FOUND ERROR")
      rospy.wait_for_service(ros_service)    
      test_service = rospy.ServiceProxy(ros_service, getUsernameAssociatedWithApplicationTokenSrv)
      req = getUsernameAssociatedWithApplicationTokenSrvRequest()
      req.application_token=app_token        
      response = test_service(req)  
      return response.username   


    ## Check if there is an active session (token) for the specified user and
    # store_token (device_token)
    #
    # @param username [string] The username
    # @param store_token [string] The device_token associated with the appl_token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_active_robot_session(self, username, store_token):
      ros_service = rospy.get_param("rapp_mysql_wrapper_check_active_application_token_service_topic")
      if(not ros_service):
        rospy.logerror("rapp_mysql_wrapper_check_active_application_token_service_topic NOT FOUND ERROR")
      rospy.wait_for_service(ros_service)    
      test_service = rospy.ServiceProxy(ros_service, checkActiveApplicationTokenSrv)
      req = checkActiveApplicationTokenSrvRequest()
      req.username=username
      req.device_token=store_token        
      response = test_service(req)     
      if(response.success):
        return response.application_token_exists

    ## Write new user to the platform db
    #
    # @param username [string] The username
    # @param password [string] The user's password
    # @param store_token [string] The user's store token
    # @param language [string] The user's language
    #
    # @return status [bool] True if token exists, false otherwise
    def add_new_user(self, username, password, store_token, language):
      ros_service = rospy.get_param("rapp_mysql_wrapper_create_new_platform_user_service_topic")
      if(not ros_service):
        rospy.logerror("rapp_mysql_wrapper_create_new_platform_user_service_topic NOT FOUND ERROR")
      rospy.wait_for_service(ros_service)    
      test_service = rospy.ServiceProxy(ros_service, createNewPlatformUserSrv)
      req = createNewPlatformUserSrvRequest()
      req.username=username
      req.password=password
      req.store_token=store_token
      req.language=language
      response = test_service(req)     
      if(response.success):
        print "success"


    ## Write new token to the platform db
    #
    # @param username [string] The user issuing the token
    # @param store_token [string] The device_token of the application token
    # @param app_token [string] The user's application token

    def write_new_application_token(self, username, store_token, appl_token):
      ros_service = rospy.get_param("rapp_mysql_wrapper_create_new_application_token_service_topic")
      if(not ros_service):
        rospy.logerror("rapp_mysql_wrapper_create_new_application_token_service_topic NOT FOUND ERROR")
      rospy.wait_for_service(ros_service)    
      test_service = rospy.ServiceProxy(ros_service, createNewApplicationTokenSrv)
      req = createNewApplicationTokenSrvRequest()
      req.username=username
      req.password=password
      req.store_token=store_token
      req.language=language
      response = test_service(req)     
      if(response.success):
        print "success"
