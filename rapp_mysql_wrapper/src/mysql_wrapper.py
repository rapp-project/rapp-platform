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
import MySQLdb as mdb
import sys

from rapp_platform_ros_communications.srv import (
  addStoreTokenToDeviceSrv,
  addStoreTokenToDeviceSrvResponse,
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
  checkActiveRobotSessionSrv,
  checkActiveRobotSessionSrvRequest,
  checkActiveRobotSessionSrvResponse,
  checkActiveApplicationTokenSrv,
  checkActiveApplicationTokenSrvRequest,
  checkActiveApplicationTokenSrvResponse,
  validateUserRoleSrv,
  validateUserRoleSrvResponse,
  validateUserRoleSrvRequest,
  validateExistingPlatformDeviceTokenSrv,
  validateExistingPlatformDeviceTokenSrvResponse,
  validateExistingPlatformDeviceTokenSrvRequest,
  removePlatformUserSrv,
  removePlatformUserSrvRequest,
  removePlatformUserSrvResponse,
  createNewCloudAgentServiceSrv,
  createNewCloudAgentServiceSrvRequest,
  createNewCloudAgentServiceSrvResponse,
  createNewCloudAgentSrv,
  createNewCloudAgentSrvRequest,
  createNewCloudAgentSrvResponse,
  getCloudAgentServiceTypeAndHostPortSrv,
  getCloudAgentServiceTypeAndHostPortSrvRequest,
  getCloudAgentServiceTypeAndHostPortSrvResponse,
  addUserEmailSrv,
  addUserEmailSrvRequest,
  addUserEmailSrvResponse,
  getPlatformUserInfoSrv,
  getPlatformUserInfoSrvRequest,
  getPlatformUserInfoSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  PlatformUserInfoMsg,
  StringArrayMsg
  )
  
## @class MySQLdbWrapper
# @brief The mysql wrapper ros node
class MySQLdbWrapper:

  ## @brief Default contructor
  #
  # Declares the callbacks of the node's services
  def __init__(self):

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_get_user_ontology_alias_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_get_user_ontology_alias_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, getUserOntologyAliasSrv, self.getUserOntologyAliasDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_register_user_ontology_alias_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_register_user_ontology_alias_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, registerUserOntologyAliasSrv, self.registerUserOntologyAliasDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_get_user_language_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_get_user_language_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, getUserLanguageSrv, self.getUserLanguageDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_check_if_user_exists_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_check_if_user_exists_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, checkIfUserExistsSrv, self.checkIfUserExistsDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_get_user_password_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_get_user_password_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, getUserPasswordSrv, self.getUserPasswordDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_get_username_associated_with_application_token_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_get_username_associated_with_application_token_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, getUsernameAssociatedWithApplicationTokenSrv, self.getUsernameAssociatedWithApplicationTokenDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_create_new_platform_user_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_create_new_platform_user_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, createNewPlatformUserSrv, self.createNewPlatformUserDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_create_new_application_token_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_create_new_application_token_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, createNewApplicationTokenSrv, self.createNewApplicationTokenDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_validate_existing_platform_device_token_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_validate_existing_platform_device_token_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, validateExistingPlatformDeviceTokenSrv, self.validateExistingPlatformDeviceTokenDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_check_active_application_token_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_check_active_application_token_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, checkActiveApplicationTokenSrv, self.checkActiveApplicationTokenDataHandler)
    
    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_remove_platform_user_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_remove_platform_user_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, removePlatformUserSrv, self.removePlatformUserDataHandler)    

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_check_active_robot_session_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_check_active_robot_session_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, checkActiveRobotSessionSrv, self.checkActiveRobotSessionDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_add_store_token_to_device_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_add_store_token_to_device_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, addStoreTokenToDeviceSrv, self.addStoreTokenToDeviceDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_validate_user_role_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_validate_user_role_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, validateUserRoleSrv, self.validateUserRoleDataHandler)
    
    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_create_new_cloud_agent_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_create_new_cloud_agent_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, createNewCloudAgentSrv, self.createNewCloudAgentDataHandler)

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_create_new_cloud_agent_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_create_new_cloud_agent_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, createNewCloudAgentServiceSrv, self.createNewCloudAgentServiceDataHandler)    

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_get_cloud_agent_service_type_and_host_port_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_get_cloud_agent_service_type_and_host_port_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, getCloudAgentServiceTypeAndHostPortSrv, self.getCloudAgentServiceTypeAndHostPortDataHandler) 
    
    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_add_user_email_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_add_user_email_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, addUserEmailSrv, self.addUserEmailSrvDataHandler)
    
    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_get_platform_user_info_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_get_platform_user_info_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, getPlatformUserInfoSrv, self.getPlatformUserInfoSrvDataHandler)   
    
  ## @brief Implements the getUserOntologyAlias service main function
  # @param req [rapp_platform_ros_communications::getUserOntologyAliasSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::getUserOntologyAliasSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError   
  def getUserOntologyAlias(self,req):
    try:
      res = getUserOntologyAliasSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select ontology_alias from platform_user where username=%s",(req.username))
      result_set = cur.fetchall()
      #print result_set
      if(result_set and len(result_set[0])>0):
        res.ontology_alias=str(result_set[0][0])
        res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the registerUserOntologyAlias service main function
  # @param req [rapp_platform_ros_communications::registerUserOntologyAliasSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::registerUserOntologyAliasSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def registerUserOntologyAlias(self,req):
    try:
      res = registerUserOntologyAliasSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("LOCK TABLES platform_user WRITE")
      cur.execute("update platform_user set ontology_alias=%s where username=%s",(req.ontology_alias,req.username))
      cur.execute("UNLOCK TABLES")
      result_set = cur.fetchall()
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the getUserLanguage service main function
  # @param req [rapp_platform_ros_communications::getUserLanguageSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::getUserLanguageSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def getUserLanguage(self,req):
    try:
      res = getUserLanguageSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select name from language where id=(select language_id from platform_user where username=%s)",(req.username))
      result_set = cur.fetchall()
      if(result_set and len(result_set[0])>0):
        res.user_language=result_set[0][0]
        res.success=True
      else:
        res.success=False
        res.error="User language is NULL"
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the checkIfUserExists service main function
  # @param req [rapp_platform_ros_communications::checkIfUserExistsSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::checkIfUserExistsSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def checkIfUserExists(self,req):
    try:
      res = checkIfUserExistsSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select username from platform_user where username=%s",(req.username))
      result_set = cur.fetchall()
      res.user_exists=False
      if(result_set and len(result_set[0])>0):
        res.user_exists=True
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the getUserPassword service main function
  # @param req [rapp_platform_ros_communications::getUserPasswordSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::getUserPasswordSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def getUserPassword(self,req):
    try:
      res = getUserPasswordSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select password from platform_user where username=%s",(req.username))
      result_set = cur.fetchall()
      res.success=False
      if(result_set and len(result_set[0])>0):
        res.success=True
        res.password=result_set[0][0]
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the getUsernameAssociatedWithApplicationToken service main function
  # @param req [rapp_platform_ros_communications::getUsernameAssociatedWithApplicationTokenSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::getUsernameAssociatedWithApplicationTokenSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def getUsernameAssociatedWithApplicationToken(self,req):
    try:
      res = getUsernameAssociatedWithApplicationTokenSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select username from platform_user where id=(select platform_user_id from application_token where token=%s)",(req.application_token))
      result_set = cur.fetchall()
      if(result_set and len(result_set[0])>0):
        res.success=True
        res.username=result_set[0][0]
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the createNewPlatformUser service main function
  # @param req [rapp_platform_ros_communications::createNewPlatformUserSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::createNewPlatformUserSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def createNewPlatformUser(self,req):
    try:
      res = createNewPlatformUserSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()    
      cur.execute("LOCK TABLES platform_user WRITE, platform_user as p2 READ, language READ")       
      cur.execute("insert into `platform_user` (`username`,`password`,`language_id`,`creator_id`) values (%s, %s, (select `id` from `language` where `name`=%s), (select `id` from platform_user as p2 where `username`=%s))",(req.username,req.password,req.language,req.creator_username)) 
      cur.execute("UNLOCK TABLES")
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the removePlatformUser service main function
  # @param req [rapp_platform_ros_communications::removePlatformUserSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::removePlatformUserSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def removePlatformUser(self,req):
    try:
      res = removePlatformUserSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("LOCK TABLES platform_user WRITE")
      cur.execute("delete from platform_user where username=%s",(req.username))
      cur.execute("UNLOCK TABLES")
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the createNewApplicationToken service main function
  # @param req [rapp_platform_ros_communications::createNewApplicationTokenSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::createNewApplicationTokenSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def createNewApplicationToken(self,req):
    try:
      res = createNewApplicationTokenSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()    
      cur.execute("LOCK TABLES application_token WRITE, platform_user READ, device READ")       
      cur.execute("insert into application_token (token,platform_user_id,device_id) VALUES (%s,(select id from platform_user where username=%s), (select id from device where token=%s))",(req.application_token,req.username,req.store_token)) 
      cur.execute("UNLOCK TABLES")
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the addStoreTokenToDevice service main function
  # @param req [rapp_platform_ros_communications::addStoreTokenToDeviceSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::addStoreTokenToDeviceSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def addStoreTokenToDevice(self,req):
    try:
      res = addStoreTokenToDeviceSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()           
      cur.execute("LOCK TABLES device WRITE") 
      cur.execute("insert into device (token, description) values (%s,'rapp store device') on duplicate key update token=token",(req.store_token))   
      cur.execute("UNLOCK TABLES")
      result_set = cur.fetchall()
      res.error = ''
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:      
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res 

  ## @brief Implements the validateUserRole service main function
  # @param req [rapp_platform_ros_communications::validateUserRoleSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::validateUserRoleSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def validateUserRole(self,req):
    try:
      res = validateUserRoleSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()           
      cur.execute("select username from platform_user where username=%s and role <= 10",(req.username))   
      result_set = cur.fetchall()
      res.error = 'Invalid role'
      if(result_set and len(result_set[0])>0):
        res.error = ''
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:      
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res 

  ## @brief Implements the checkActiveRobotSession service main function
  # @param req [rapp_platform_ros_communications::checkActiveRobotSessionSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::checkActiveRobotSessionSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def checkActiveRobotSession(self,req):
    try:
      res = checkActiveRobotSessionSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()           
      cur.execute("select token from application_token where platform_user_id=(select id from platform_user where username=%s) and status=1 and device_id=(select id from device where token=%s)",(req.username,req.device_token))   
      result_set = cur.fetchall()
      res.application_token_exists=False
      if(result_set and len(result_set[0])>0):
        res.application_token_exists=True        
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:      
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res 

  ## @brief Implements the checkActiveRobotSession service main function
  # @param req [rapp_platform_ros_communications::checkActiveRobotSessionSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::checkActiveRobotSessionSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError
  def checkActiveApplicationToken(self,req):
    try:
      res = checkActiveApplicationTokenSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()           
      cur.execute("select token from application_token where token=%s and status=1",(req.application_token))   
      result_set = cur.fetchall()
      res.application_token_exists=False
      if(result_set and len(result_set[0])>0):
        res.application_token_exists=True
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res

  ## @brief Implements the validateExistingPlatformDeviceToken service main function
  # @param req [rapp_platform_ros_communications::validateExistingPlatformDeviceTokenSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::validateExistingPlatformDeviceTokenSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError
  def validateExistingPlatformDeviceToken(self,req):
    try:
      res = validateExistingPlatformDeviceTokenSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()           
      cur.execute("select token from device where token=%s and status=1",(req.device_token))   
      result_set = cur.fetchall()
      res.device_token_exists=False
      res.success=False
      if(result_set and len(result_set[0])>0):
        res.device_token_exists=True        
        res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:      
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res 

  ## @brief Implements the createNewCloudAgent service main function
  # @param req [rapp_platform_ros_communications::createNewCloudAgentSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::createNewCloudAgentSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError
  def createNewCloudAgent(self,req):
    try:
      res = createNewCloudAgentSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()    
      cur.execute("LOCK TABLES cloud_agent WRITE, platform_user READ")       
      cur.execute("insert into cloud_agent (platform_user_id,tarball_path,container_identifier,image_identifier,container_type) VALUES ((select id from platform_user where username=%s), %s,%s,%s,%s)",(req.username,req.tarball_path,req.container_identifier,req.image_identifier,req.container_type)) 
      cur.execute("UNLOCK TABLES")
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res   

  ## @brief Implements the createNewCloudAgentService service main function
  # @param req [rapp_platform_ros_communications::createNewCloudAgentServiceSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::createNewCloudAgentServiceSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError
  def createNewCloudAgentService(self,req):
    try:
      res = createNewCloudAgentServiceSrvResponse()   
      print "right"     
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()    
      cur.execute("LOCK TABLES cloud_agent READ, cloud_agent_services WRITE")     
      cur.execute("insert into cloud_agent_services (cloud_agent_id,service_name,service_type,container_port,host_port) VALUES ((select id from cloud_agent where container_identifier=%s), %s,%s,%s,%s)",(req.container_identifier,req.service_name,req.service_type,req.container_port,req.host_port)) 
      cur.execute("UNLOCK TABLES")
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res 

  ## @brief Implements the getCloudAgentServiceTypeAndHostPort service main function
  # @param req [rapp_platform_ros_communications::getCloudAgentServiceTypeAndHostPortSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::getCloudAgentServiceTypeAndHostPortSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError
  def getCloudAgentServiceTypeAndHostPort(self,req):
    try:
      res = getCloudAgentServiceTypeAndHostPortSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select service_type,host_port from cloud_agent_services where cloud_agent_id=(select id from cloud_agent where container_identifier=%s) and service_name=%s",(req.container_identifier,req.service_name))
      result_set = cur.fetchall()
      if(result_set and len(result_set[0])>0):
        res.success=True
        res.service_type=result_set[0][0]
        res.host_port=result_set[0][1]
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res


  ## @brief Implements the addUserEmailService service main function
  # @param req [rapp_platform_ros_communications::addUserEmailSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::addUserEmailSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError
  def addUserEmailService(self,req):
    try:
      res = addUserEmailSrvResponse()             
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()    
      cur.execute("LOCK TABLES platform_user READ, platform_user_emails WRITE")     
      cur.execute("insert into platform_user_emails (platform_user_id,owner,email) VALUES ((select id from platform_user where username=%s),%s,%s)",(req.username,req.owner,req.email)) 
      cur.execute("UNLOCK TABLES")
      res.success=True
      con.close()
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res 

  ## @brief Implements the getPlatformUserInfoService service main function
  # @param req [rapp_platform_ros_communications::getPlatformUserInfoSrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::getPlatformUserInfoSrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception mdbError 
  def getPlatformUserInfoService(self,req):
    try:
      res = getPlatformUserInfoSrvResponse()      
      userInfo = PlatformUserInfoMsg()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select name,surname,ontology_alias,(select name from language where id=language_id),id from platform_user where username=%s",(req.username))
      result_set = cur.fetchall()
      if(result_set and len(result_set[0])>0):
        userInfo.name=result_set[0][0]
        userInfo.surname=result_set[0][1]   
        userInfo.ontology_alias=result_set[0][2]  
        userInfo.language=result_set[0][3]     
        userid=result_set[0][4]                
        cur.execute("select owner,email from platform_user_emails where platform_user_id=%s",(userid))
        result_set = cur.fetchall()
        for i in range(len(result_set)):
          emailEntry=StringArrayMsg()
          for j in range(len(result_set[i])):
            temp_s=result_set[i][j]
            emailEntry.s.append((str(result_set[i][j])))
          userInfo.emails.append(emailEntry) 

      res.platform_user_info=userInfo
      con.close()
      res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
      con.close()
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
      con.close()
    except IOError, e:
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
      con.close()
    return res


  ## @brief Gets the columns of the table
  # @param tblName [string] the name of the table
  #
  # @return Columns [list] the columns of the table
  def getTableColumnNames(self,tblName):
    db_username,db_password=self.getLogin()
    try:
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("Show columns from "+tblName)
      result_set = cur.fetchall()
      Columns=[]
      for row in result_set:
        Columns=Columns+[String(str(row[0]))]
      return Columns
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])

  ## @brief Loads login details from file
  # @return db_username [string] the login username
  # @return db_password [string] the login password
  def getLogin(self):
    fh = open("/etc/db_credentials", "r")
    db_username=fh.readline()
    db_username=db_username.split( )[0]
    db_password=fh.readline()
    db_password=db_password.split()[0]
    return db_username,db_password

  ## @brief Checks connectivity to the DB
  # @return [ void ] Connection performed successfully
  def checkConnection(self):
    try:
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform')
      cur = con.cursor()
      cur.execute("SELECT VERSION()")
      ver = cur.fetchone()
      print "Database version : %s " % ver
      con.close()
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])

  ## @brief The getUserOntologyAlias service callback
  # @param req [rapp_platform_ros_communications::getUserOntologyAliasSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::getUserOntologyAliasSrvRequest::Response&] The ROS service response
  def getUserOntologyAliasDataHandler(self,req):
    res = getUserOntologyAliasSrvResponse()
    res=self.getUserOntologyAlias(req)
    return res

  ## @brief The registerUserOntologyAliasSrv service callback
  # @param req [rapp_platform_ros_communications::registerUserOntologyAliasSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::registerUserOntologyAliasSrvRequest::Response&] The ROS service response
  def registerUserOntologyAliasDataHandler(self,req):
    res = registerUserOntologyAliasSrvResponse()
    res=self.registerUserOntologyAlias(req)
    return res

  ## @brief The getUserLanguage service callback
  # @param req [rapp_platform_ros_communications::getUserLanguageSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::getUserLanguageSrvRequest::Response&] The ROS service response
  def getUserLanguageDataHandler(self,req):
    res = getUserLanguageSrvResponse()
    res=self.getUserLanguage(req)
    return res

  ## @brief The registerNewTokenSrv service callback
  # @param req [rapp_platform_ros_communications::registerNewTokenSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::registerNewTokenSrvRequest::Response&] The ROS service response
  def registerNewTokenDataHandler(self,req):
    res = registerNewTokenSrvResponse()
    res=self.registerNewToken(req)
    return res

  ## @brief The checkIfUserExistsSrv service callback
  # @param req [rapp_platform_ros_communications::checkIfUserExistsSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::checkIfUserExistsSrvRequest::Response&] The ROS service response
  def checkIfUserExistsDataHandler(self,req):
    res = checkIfUserExistsSrvResponse()
    res=self.checkIfUserExists(req)
    return res

  ## @brief The getUserPasswordSrv service callback
  # @param req [rapp_platform_ros_communications::getUserPasswordSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::getUserPasswordSrvRequest::Response&] The ROS service response
  def getUserPasswordDataHandler(self,req):
    res = getUserPasswordSrvResponse()
    res=self.getUserPassword(req)
    return res

  ## @brief The getUsernameAssociatedWithApplicationTokenSrv service callback
  # @param req [rapp_platform_ros_communications::getUsernameAssociatedWithApplicationTokenSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::getUsernameAssociatedWithApplicationTokenSrvRequest::Response&] The ROS service response
  def getUsernameAssociatedWithApplicationTokenDataHandler(self,req):
    res = getUsernameAssociatedWithApplicationTokenSrvResponse()
    res=self.getUsernameAssociatedWithApplicationToken(req)
    return res

  ## @brief The createNewPlatformUserSrv service callback
  # @param req [rapp_platform_ros_communications::createNewPlatformUserSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::createNewPlatformUserSrvRequest::Response&] The ROS service response
  def createNewPlatformUserDataHandler(self,req):
    res = createNewPlatformUserSrvResponse()
    res=self.createNewPlatformUser(req)
    return res

  ## @brief The createNewApplicationTokenSrv service callback
  # @param req [rapp_platform_ros_communications::createNewApplicationTokenSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::createNewApplicationTokenSrvRequest::Response&] The ROS service response
  def createNewApplicationTokenDataHandler(self,req):
    res = createNewApplicationTokenSrvResponse()
    res=self.createNewApplicationToken(req)
    return res

  ## @brief The checkActiveApplicationTokenSrv service callback
  # @param req [rapp_platform_ros_communications::checkActiveApplicationTokenSrvResponse::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::checkActiveApplicationTokenSrvRequest::Response&] The ROS service response
  def checkActiveApplicationTokenDataHandler(self,req):
    res = checkActiveApplicationTokenSrvResponse()
    res=self.checkActiveApplicationToken(req)
    return res

  ## @brief The checkActiveRobotSessionSrv service callback
  # @param req [rapp_platform_ros_communications::checkActiveRobotSessionSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::checkActiveRobotSessionSrvResponse::Response&] The ROS service response
  def checkActiveRobotSessionDataHandler(self,req):
    res = checkActiveRobotSessionSrvResponse()
    res=self.checkActiveRobotSession(req)
    return res

  ## @brief The addStoreTokenToDeviceSrv service callback
  # @param req [rapp_platform_ros_communications::addStoreTokenToDeviceSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::addStoreTokenToDeviceSrvResponse::Response&] The ROS service response
  def addStoreTokenToDeviceDataHandler(self,req):
    res = addStoreTokenToDeviceSrvResponse()
    res=self.addStoreTokenToDevice(req)
    return res

  ## @brief The validateUserRoleSrv service callback
  # @param req [rapp_platform_ros_communications::validateUserRoleSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::validateUserRoleSrvResponse::Response&] The ROS service response
  def validateUserRoleDataHandler(self,req):
    res = validateUserRoleSrvResponse()
    res=self.validateUserRole(req)
    return res

  ## @brief The validateExistingPlatformDeviceTokenSrv service callback
  # @param req [rapp_platform_ros_communications::validateExistingPlatformDeviceTokenSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::validateExistingPlatformDeviceTokenSrvResponse::Response&] The ROS service response
  def validateExistingPlatformDeviceTokenDataHandler(self,req):
    res = validateExistingPlatformDeviceTokenSrvResponse()
    res=self.validateExistingPlatformDeviceToken(req)
    return res

  ## @brief The removePlatformUserSrv service callback
  # @param req [rapp_platform_ros_communications::removePlatformUserSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::removePlatformUserSrvResponse::Response&] The ROS service response
  def removePlatformUserDataHandler(self,req):
    res = removePlatformUserSrvResponse()
    res=self.removePlatformUser(req)
    return res

  ## @brief The createNewCloudAgentSrv service callback
  # @param req [rapp_platform_ros_communications::createNewCloudAgentSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::createNewCloudAgentSrvResponse::Response&] The ROS service response
  def createNewCloudAgentDataHandler(self,req):
    res = createNewCloudAgentSrvResponse()
    res=self.createNewCloudAgent(req)
    return res

  ## @brief The createNewCloudAgentServiceSrv service callback
  # @param req [rapp_platform_ros_communications::createNewCloudAgentServiceSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::createNewCloudAgentServiceSrvResponse::Response&] The ROS service response
  def createNewCloudAgentServiceDataHandler(self,req):
    res = createNewCloudAgentServiceSrvResponse()
    res=self.createNewCloudAgentService(req)
    return res

  ## @brief The getCloudAgentServiceTypeAndHostPortSrv service callback
  # @param req [rapp_platform_ros_communications::getCloudAgentServiceTypeAndHostPortSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::getCloudAgentServiceTypeAndHostPortSrvResponse::Response&] The ROS service response
  def getCloudAgentServiceTypeAndHostPortDataHandler(self,req):
    res = getCloudAgentServiceTypeAndHostPortSrvResponse()
    res=self.getCloudAgentServiceTypeAndHostPort(req)
    return res

  ## @brief The addUserEmailSrv service callback
  # @param req [rapp_platform_ros_communications::addUserEmailSrvDataHandlerRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::addUserEmailSrvDataHandlerResponse::Response&] The ROS service response
  def addUserEmailSrvDataHandler(self,req):
    res = addUserEmailSrvResponse()
    res=self.addUserEmailService(req)
    return res

  ## @brief The getPlatformUserInfoSrv service callback
  # @param req [rapp_platform_ros_communications::getPlatformUserInfoSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::getPlatformUserInfoSrvResponse::Response&] The ROS service response
  def getPlatformUserInfoSrvDataHandler(self,req):
    res = getPlatformUserInfoSrvResponse()
    res=self.getPlatformUserInfoService(req)
    return res
