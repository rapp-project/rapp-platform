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
  validateExistingPlatformDeviceTokenSrvRequest
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from std_msgs.msg import (
  String
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

  def getUserLanguage(self,req):
    try:
      res = getUserLanguageSrvResponse()
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'rapp_platform');
      cur = con.cursor()
      cur.execute("select language from platform_user where username=%s",(req.username))
      result_set = cur.fetchall()
      if(result_set and len(result_set[0])>0):
        res.user_language=result_set[0][0]
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

  ## @brief Gets the columns of the table
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
  # @return [ void ] Login details loaded successfully
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
  # @param res [rapp_platform_ros_communications::getUserOntologyAliasSrvRequest::Response&] The ROS service response
  def getUserOntologyAliasDataHandler(self,req):
    res = getUserOntologyAliasSrvResponse()
    res=self.getUserOntologyAlias(req)
    return res

  ## @brief The registerUserOntologyAliasSrv service callback
  # @param req [rapp_platform_ros_communications::registerUserOntologyAliasSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::registerUserOntologyAliasSrvRequest::Response&] The ROS service response
  def registerUserOntologyAliasDataHandler(self,req):
    res = registerUserOntologyAliasSrvResponse()
    res=self.registerUserOntologyAlias(req)
    return res

  ## @brief The getUserLanguage service callback
  # @param req [rapp_platform_ros_communications::getUserLanguageSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::getUserLanguageSrvRequest::Response&] The ROS service response
  def getUserLanguageDataHandler(self,req):
    res = getUserLanguageSrvResponse()
    res=self.getUserLanguage(req)
    return res

  ## @brief The registerNewTokenSrv service callback
  # @param req [rapp_platform_ros_communications::registerNewTokenSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::registerNewTokenSrvRequest::Response&] The ROS service response
  def registerNewTokenDataHandler(self,req):
    res = registerNewTokenSrvResponse()
    res=self.registerNewToken(req)
    return res

  ## @brief The checkIfUserExistsSrv service callback
  # @param req [rapp_platform_ros_communications::checkIfUserExistsSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::checkIfUserExistsSrvRequest::Response&] The ROS service response
  def checkIfUserExistsDataHandler(self,req):
    res = checkIfUserExistsSrvResponse()
    res=self.checkIfUserExists(req)
    return res

  ## @brief The getUserPasswordSrv service callback
  # @param req [rapp_platform_ros_communications::getUserPasswordSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::getUserPasswordSrvRequest::Response&] The ROS service response
  def getUserPasswordDataHandler(self,req):
    res = getUserPasswordSrvResponse()
    res=self.getUserPassword(req)
    return res

  ## @brief The getUsernameAssociatedWithApplicationTokenSrv service callback
  # @param req [rapp_platform_ros_communications::getUsernameAssociatedWithApplicationTokenSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::getUsernameAssociatedWithApplicationTokenSrvRequest::Response&] The ROS service response
  def getUsernameAssociatedWithApplicationTokenDataHandler(self,req):
    res = getUsernameAssociatedWithApplicationTokenSrvResponse()
    res=self.getUsernameAssociatedWithApplicationToken(req)
    return res

  ## @brief The createNewPlatformUserSrv service callback
  # @param req [rapp_platform_ros_communications::createNewPlatformUserSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::createNewPlatformUserSrvRequest::Response&] The ROS service response
  def createNewPlatformUserDataHandler(self,req):
    res = createNewPlatformUserSrvResponse()
    res=self.createNewPlatformUser(req)
    return res

  ## @brief The createNewApplicationTokenSrv service callback
  # @param req [rapp_platform_ros_communications::createNewApplicationTokenSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::createNewApplicationTokenSrvRequest::Response&] The ROS service response
  def createNewApplicationTokenDataHandler(self,req):
    res = createNewApplicationTokenSrvResponse()
    res=self.createNewApplicationToken(req)
    return res

  ## @brief The checkActiveApplicationTokenSrv service callback
  # @param req [rapp_platform_ros_communications::checkActiveApplicationTokenSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::checkActiveApplicationTokenSrvRequest::Response&] The ROS service response
  def checkActiveApplicationTokenDataHandler(self,req):
    res = checkActiveApplicationTokenSrvResponse()
    res=self.checkActiveApplicationToken(req)
    return res

  def checkActiveRobotSessionDataHandler(self,req):
    res = checkActiveRobotSessionSrvResponse()
    res=self.checkActiveRobotSession(req)
    return res

  def addStoreTokenToDeviceDataHandler(self,req):
    res = addStoreTokenToDeviceSrvResponse()
    res=self.addStoreTokenToDevice(req)
    return res

  def validateUserRoleDataHandler(self,req):
    res = validateUserRoleSrvResponse()
    res=self.validateUserRole(req)
    return res

  def validateExistingPlatformDeviceTokenDataHandler(self,req):
    res = validateExistingPlatformDeviceTokenSrvResponse()
    res=self.validateExistingPlatformDeviceToken(req)
    return res

