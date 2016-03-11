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
  authTokenByServiceSrv,
  authTokenByServiceSrvResponse,
  getUserOntologyAliasSrv,
  getUserOntologyAliasSrvResponse,
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
    
    
    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_auth_token_by_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_auth_token_by_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, authTokenByServiceSrv, self.authTokenByServiceDataHandler)    
        
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
 
    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_register_new_token_service_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_register_new_token_service_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, registerNewTokenServiceSrv, self.registerNewTokenServiceDataHandler) 

    self.serv_topic = rospy.get_param("rapp_mysql_wrapper_get_services_by_token_service_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_mysql_wrapper_get_services_by_token_service_topic Not found error")
    self.serv=rospy.Service(self.serv_topic, getServicesByTokenSrv, self.getServicesByTokenDataHandler) 
    

  def getUserOntologyAlias(self,req):
    try:
      res = getUserOntologyAliasSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
      cur = con.cursor()
      cur.execute("select ontology_alias from users where id=%s",(req.user_id))   
      result_set = cur.fetchall()
      #print result_set
      if(result_set and len(result_set[0])>0): 
        res.ontology_alias=result_set[0][0]
        res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
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
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
      cur = con.cursor()
      cur.execute("LOCK TABLES users WRITE") 
      cur.execute("update users set ontology_alias=%s where id=%s",(req.ontology_alias,req.user_id))   
      cur.execute("UNLOCK TABLES")
      result_set = cur.fetchall()
      res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
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
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
      cur = con.cursor()      
      cur.execute("select language from users where id=%s",(req.user_id))  
      result_set = cur.fetchall()
      if(result_set and len(result_set[0])>0): 
        res.user_language=result_set[0][0]
        res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
    except IOError, e:      
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
    con.close()
    return res 

  def registerNewToken(self,req):
    try:
      res = registerNewTokenSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
      cur = con.cursor()
      cur.execute("LOCK TABLES application_token WRITE") 
      cur.execute("insert into application_token (token,robot_id) values(%s,%s)",(req.token,req.robot_id))   
      cur.execute("UNLOCK TABLES")
      result_set = cur.fetchall()
      res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
    except IOError, e:      
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
    con.close()
    return res     

  def registerNewTokenService(self,req):
    try:
      res = registerNewTokenServiceSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
      cur = con.cursor()
      cur.execute("LOCK TABLES application_token_services WRITE, application_token WRITE") 
      cur.execute("insert into application_token_services values((select id from application_token where token=%s),%s)",(req.token,req.service_name))   
      cur.execute("UNLOCK TABLES")
      result_set = cur.fetchall()
      res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
    except IOError, e:      
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
    con.close()
    return res 

  def authTokenByService(self,req):
    try:
      res = authTokenByServiceSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
      cur = con.cursor()           
      cur.execute("select service_name from application_token_services where token_id=(select id from application_token where token=%s) and (service_name=%s or service_name='*')",(req.token,req.service_name))   
      result_set = cur.fetchall()
      if(result_set and len(result_set[0])>0):
        if(result_set[0][0]==req.service_name):
          res.authentication_success=True
      else:
        res.authentication_success=False       
      res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
    except IOError, e:      
      res.success=False
      res.trace.append("IOError: " +str(e))
      res.error="IOError: " +str(e)
    con.close()
    return res  
    
  def getServicesByToken(self,req):
    try:
      res = getServicesByTokenSrvResponse()        
      db_username,db_password=self.getLogin()
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
      cur = con.cursor()           
      cur.execute("select service_name from application_token_services where token_id=(select token_id from application_token where token=%s)",(req.token))   
      result_set = cur.fetchall()
      for i in range(len(result_set)):
        for j in range(len(result_set[i])):   
          res.services.append((str(result_set[i][j])))      
      res.success=True
    except mdb.Error, e:
      res.trace.append(("Database Error %d: %s" % (e.args[0],e.args[1])))
      res.success=False
      res.error="Error %d: %s" % (e.args[0],e.args[1])
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.success=False
      res.error="IndexError: " +str(e)
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
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');
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
      con = mdb.connect('localhost', db_username, db_password, 'RappStore')
      cur = con.cursor()
      cur.execute("SELECT VERSION()")
      ver = cur.fetchone()
      print "Database version : %s " % ver
      con.close()
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])

  ## @brief The authTokenByService service callback
  # @param req [rapp_platform_ros_communications::authTokenByServiceSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::authTokenByServiceSrvRequest::Response&] The ROS service response
  def authTokenByServiceDataHandler(self,req):
    res = authTokenByServiceSrvResponse()
    res=self.authTokenByService(req)
    return res
    
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

  ## @brief The registerNewTokenServiceSrv service callback
  # @param req [rapp_platform_ros_communications::registerNewTokenServiceSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::registerNewTokenServiceSrvRequest::Response&] The ROS service response
  def registerNewTokenServiceDataHandler(self,req):
    res = registerNewTokenServiceSrvResponse()
    res=self.registerNewTokenService(req)
    return res
    
  ## @brief The getServicesByTokenSrv service callback
  # @param req [rapp_platform_ros_communications::getServicesByTokenSrvResponse::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::getServicesByTokenSrvRequest::Response&] The ROS service response
  def getServicesByTokenDataHandler(self,req):
    res = getServicesByTokenSrvResponse()
    res=self.getServicesByToken(req)
    return res
