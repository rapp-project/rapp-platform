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


from std_msgs.msg import (
  String
  )

## @class TestDbWrapper 
# Inherits the unittest.TestCase class in order to offer functional tests functionality 
class TestDbWrapper(unittest.TestCase):
  ## Tests the rapp_mysql_wrapper_user_write_data service when an invalid column is provided 
  def testDBError(self):
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_write_data_topic')
    if(not serv_topic):
      rospy.logerror("rapp_mysql_wrapper_user_write_data_topic NOT FOUND ERROR")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    req = writeDataSrv()
    req.req_cols=[]#[String(data="model_str"),String(data="manufacturer"),String(data="version"),String(data="arch"),String(data="os"),String(data="picture")]
    #req.req_cols=[String("id"),String("macddr"), String("model"),String("owner"), String("timestamp")]
    req.req_cols=["idsd","macddr", "model","owner", "timestamp"]
    entry1=StringArrayMsg()
    #entry1=[string("'3'"),string("'1800000'"), string("'1'"),string("'1'"), string("'2014-11-23 09:04:13'")]
    entry1=["'3'","'1800000'", "'1'","'25'", "'2014-11-23 09:04:13'"]
    entry2=StringArrayMsg()
    #entry2=[string("'4'"),string("'1900000'"), string("'1'"),string("'1'"), string("'2014-11-23 07:04:13'")]
    entry2=["'4'","'1900000'", "'1'","'25'", "'2014-11-23 07:04:13'"]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]

    response = db_service(req.req_cols,req.req_data)
    self.assertEqual(response.trace[0],"Database Error 1054: Unknown column 'idsd' in 'field list'")
    self.assertFalse(response.success.data)

  ## Tests the rapp_mysql_wrapper_robot_fetch_data_ service when null input is provided 
  def testNullInputError(self):
    serv_topic = rospy.get_param('rapp_mysql_wrapper_robot_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_robot_read_data_topic")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    req = fetchDataSrv()
    req.req_cols=[]
    entry1=[]
    req.where_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.req_cols,req.where_data)
    self.assertEqual(response.trace[0],"IndexError: list index out of range")
    self.assertFalse(response.success.data)

  ## Tests the the write,read,update,delete rapp mysql wrapper services of the tblUser
  def testTblUserWriteReadDeleteCheck(self):
    #Write
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_write_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_user_write_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    req = writeDataSrv()
    req.req_cols=["name","email","pwd","activated","language","ontology_alias"]
    #req.req_cols=["idsd","macddr", "model","owner", "timestamp"]
    entry1=StringArrayMsg()
    entry1=["'rappMysqlTestTemp1'","'rappMysqlTestTemp1@test.com'","'rappMysqlTestTemp1TestPass'","'Y'","'el'","NULL"]
    entry2=StringArrayMsg()
    entry2=["'rappMysqlTestTemp2'","'rappMysqlTestTemp2@test.com'","'rappMysqlTestTemp2TestPass'","'Y'","'el'","NULL"]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]

    response = db_service(req.req_cols,req.req_data)
    self.assertEqual(response.trace[0],"Success")
    self.assertTrue(response.success.data)
    #Read what was written
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_user_fetch_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    req = fetchDataSrv()
    req.req_cols=["name","email"]
    entry1=["name","rappMysqlTestTemp1"]
    req.where_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.req_cols,req.where_data)
    self.assertEqual(response.trace[0],"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0],"rappMysqlTestTemp1")
    #self.assertEqual(response.res_data[1].s[0],"rappMysqlTestTemp1@test.com")
    #Update written
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_update_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_user_update_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, updateDataSrv)
    req = updateDataSrv()
    req.set_cols=["name='rappMysqlTestTemp1'"]
    entry1=["name","rappMysqlTestTemp2"]
    req.where_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.set_cols,req.where_data)
    self.assertEqual(response.trace[0],"Success")
    self.assertTrue(response.success.data)
    #Read again
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_user_read_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    req = fetchDataSrv()
    req.req_cols=["name","email"]
    entry1=["name","rappMysqlTestTemp1"]
    req.where_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.req_cols,req.where_data)
    self.assertEqual(response.trace[0],"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0],"rappMysqlTestTemp1")
    self.assertEqual(response.res_data[1].s[0],"rappMysqlTestTemp1")
    #Delete updated
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_delete_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_user_delete_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, deleteDataSrv)
    req = deleteDataSrv()
    entry1=["name","rappMysqlTestTemp1"]
    req.where_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.where_data)
    self.assertEqual(response.trace[0],"Success")
    self.assertTrue(response.success.data)
    #Check if it was deleted
    serv_topic = rospy.get_param('rapp_mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("mysql_wrapper_user_read_data topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    req = fetchDataSrv()
    req.req_cols=["name","email"]
    entry1=["name","rappMysqlTestTemp1"]
    req.where_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.req_cols,req.where_data)
    self.assertEqual(response.trace[0],"Success")
    self.assertTrue(response.success.data)
    self.assertTrue((len(response.res_data)<1))


  ### Tests the the write,read,update,delete rapp mysql wrapper services of the tblModel
  #def testTblModelWriteReadDeleteCheck(self):
    ##Write
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_model_write_data_topic')
    #if(not serv_topic):
      #rospy.logerror("rapp_mysql_wrapper_model_write_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    #req = writeDataSrv()
    #req.req_cols=["model_str","manufacturer","version","arch","os","picture"] 
    #entry1=StringArrayMsg()
    #entry1=["'testingDB1'","'testingDB1'","'10.1'","'test'","'test'","'test'"]
    #entry2=StringArrayMsg()
    #entry2=["'testingDB2'","'testingDB1'","'10.1'","'test'","'test'","'test'"]
    #req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]

    #response = db_service(req.req_cols,req.req_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    ##Read what was written
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_model_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("rapp_mysql_wrapper_model_fetch_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["model_str","manufacturer"]
    #entry1=["manufacturer","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[0],"testingDB1")
    #self.assertEqual(response.res_data[1].s[0],"testingDB2")
    ##Update written
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_model_update_data_topic')
    #if(not serv_topic):
      #rospy.logerror("rapp_mysql_wrapper_model_update_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, updateDataSrv)
    #req = updateDataSrv()
    #req.set_cols=["model_str='testingDB3'"]
    #entry1=["model_str","testingDB2"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.set_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    ##Read again
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_model_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("rapp_mysql_wrapper_model_read_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["model_str","manufacturer"]
    #entry1=["manufacturer","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[1],"testingDB1")
    #self.assertEqual(response.res_data[1].s[1],"testingDB1")
    ##Delete updated
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_model_delete_data_topic')
    #if(not serv_topic):
      #rospy.logerror("rapp_mysql_wrapper_model_delete_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, deleteDataSrv)
    #req = deleteDataSrv()
    #entry1=["manufacturer","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    ##Check if it was deleted
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_model_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("rapp_mysql_wrapper_model_read_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["model_str","manufacturer"]
    #entry1=["model_str","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertTrue((len(response.res_data)<1))

  ### Tests the the write,read,update,delete rapp mysql wrapper services of the tblRapp
  #def testTblRappWriteReadDeleteCheck(self):
    ##Write
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_rapp_write_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_rapp_write_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    #req = writeDataSrv()
    #req.req_cols=["rapp","version","arch","lang","owner","directory","enabled","timestamp"]
    #entry1=StringArrayMsg()
    #entry1=["'testingDB1'","'1.01'","'15'","'1'","'25'","'testingDB1'","'0'","'555'"]
    #entry2=StringArrayMsg()
    #entry2=["'testingDB2'","'1.01'","'15'","'1'","'25'","'testingDB1'","'0'","'555'"]
    #req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]

    #response = db_service(req.req_cols,req.req_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)

    ##Read what was written
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_rapp_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_rapp_fetch_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["rapp","directory"]
    #entry1=["directory","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[0],"testingDB1")
    #self.assertEqual(response.res_data[1].s[0],"testingDB2")
    ##Update written
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_rapp_update_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_rapp_update_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, updateDataSrv)
    #req = updateDataSrv()
    #req.set_cols=["rapp='testingDB3'"]
    #entry1=["rapp","testingDB2"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.set_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    ##Read again
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_rapp_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_rapp_read_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["rapp","directory"]
    #entry1=["directory","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[0],"testingDB1")
    #self.assertEqual(response.res_data[1].s[0],"testingDB3")
    ##Delete updated
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_rapp_delete_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_rapp_delete_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, deleteDataSrv)
    #req = deleteDataSrv()
    #entry1=["directory","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    ##Check if it was deleted
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_rapp_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_rapp_read_data topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["rapp","directory"]
    #entry1=["directory","testingDB1"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertTrue((len(response.res_data)<1))


  ### Tests the the write,read,update,delete rapp mysql wrapper services of the tblRobot
  #def testTblRobotWriteReadDeleteCheck(self):
    ##Write
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_robot_write_data_topic')
    #if(not serv_topic):
      #rospy.logerror("rapp_mysql_wrapper_robot_write_data_topic NOT FOUND ERROR")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, writeDataSrv)
    #req = writeDataSrv()
    #req.req_cols=[]#[String(data="model_str"),String(data="manufacturer"),String(data="version"),String(data="arch"),String(data="os"),String(data="picture")]
    ##req.req_cols=[String("id"),String("macddr"), String("model"),String("owner"), String("timestamp")]
    #req.req_cols=["macddr", "model","owner", "timestamp"]
    #entry1=StringArrayMsg()
    #entry1=["'1800000'", "'1'","'25'", "'2014-11-23 09:04:13'"]
    #entry2=StringArrayMsg()
    #entry2=["'1900000'", "'1'","'25'", "'2014-11-23 07:04:13'"]
    #req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]

    #response = db_service(req.req_cols,req.req_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)

    ##Read what was written
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_robot_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_robot_read_data_topic")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["macddr"]
    #entry1=["macddr","1800000"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[0],"1800000")
    ##self.assertEqual(response.res_data[1].s[0].data,"extreme3")
    ##Update written
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_robot_update_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_robot_update_data_topic")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, updateDataSrv)
    #req = updateDataSrv()
    #req.set_cols=["timestamp='2014-11-23 09:04:13'"]
    #entry1=["macddr",'1900000']
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.set_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    ##Read again
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_robot_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_robot_fetch_data_topic")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["macddr","model"]
    #entry1=["timestamp","2014-11-23 09:04:13"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[0],"1800000")
    #self.assertEqual(response.res_data[1].s[0],"1900000")
    ##Delete updated
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_robot_delete_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_robot_delete_data_topic")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, deleteDataSrv)
    #req = deleteDataSrv()

    #entry1=["timestamp","2014-11-23 09:04:13"]
    #entry2=["model","1"]
    #req.where_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    #response = db_service(req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    ##Check if it was deleted
    #serv_topic = rospy.get_param('rapp_mysql_wrapper_robot_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_robot_write_delete_topic")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=["macddr","model"]
    #entry1=["timestamp","2014-11-23 09:04:13"]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0],"Success")
    #self.assertTrue(response.success.data)
    #self.assertTrue((len(response.res_data)<1))



## The main function. Initializes the Rapp mysql wrapper functional tests
if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)

  #def testSubmitCorrectQuery(self):
    #serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("Speech detection topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    #req = DbWrapperSrv()
    #req.return_cols=[String(data="id"), String(data="username"), String(data="firstname"), String(data="email_id")]
    ##req.return_cols=[String(data="*")]
    #entry1=StringArrayMsg()
    #entry1=[String(data="username"), String(data="admin")]
    #entry2=StringArrayMsg()
    #entry2=[String(data="firstname"), String(data="Alex")]
    #req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]

    #response = db_service(req.return_cols,req.req_data)
    #print "Report "+response.report.data
    #for i in range(len(response.res_cols)):
      #sys.stdout.write(response.res_cols[i].data+" ")
    #sys.stdout.write('\n')
    #for i in range(len(response.res_data)):
      #for j in range(len(response.res_data[i].s)):
        #sys.stdout.write(response.res_data[i].s[j].data+" ")
      #sys.stdout.write('\n')
    #sys.stdout.write('\n')
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.report.data,"Success")
    #self.assertEqual(response.res_cols[0].data,"id")
    #self.assertEqual(response.res_cols[1].data,"username")
    #self.assertEqual(response.res_cols[2].data,"firstname")
    #self.assertEqual(response.res_cols[3].data,"email_id")
    #self.assertEqual(response.res_data[0].s[0].data,"1")
    #self.assertEqual(response.res_data[0].s[1].data,"admin")
    #self.assertEqual(response.res_data[0].s[3].data,"0")

  #def testSubmitIncompleteQuery(self):
    #serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("Speech detection topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    #req = DbWrapperSrv()
    #req.return_cols=[]#[String(data="id"), String(data="username"), String(data="firstname"), String(data="otinanai")]
    #entry1=StringArrayMsg()
    ##entry1=[]
    #entry1=[String(data="username")]
    #entry2=StringArrayMsg()
    #entry2=[]
    #req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    #response = db_service(req.return_cols,req.req_data)
    #self.assertFalse(response.success.data)
    #self.assertEqual(response.report.data,"Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format")

  #def testSubmitWrongQuery(self):
    #serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("Speech detection topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    #req = DbWrapperSrv()
    #req.return_cols=[String(data="id"), String(data="username"), String(data="firstname"), String(data="otinanai")]
    #entry1=StringArrayMsg()
    #entry1=[String(data="username"), String(data="admin")]
    #entry2=StringArrayMsg()
    #entry2=[String(data="firstname"), String(data="Alex")]
    #req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    #response = db_service(req.return_cols,req.req_data)
    #self.assertFalse(response.success.data)
    #errorStart=response.report.data[0:14]
    #print errorStart
    #self.assertEqual(errorStart,"Database Error")

  #def testSubmitCorrectReturnAllColumnQuery(self):
    #serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("Speech detection topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    #req = DbWrapperSrv()
    #req.return_cols=[String(data="*")]
    #entry1=StringArrayMsg()
    #entry1=[String(data="username"), String(data="admin")]
    #entry2=StringArrayMsg()
    #entry2=[String(data="firstname"), String(data="Alex")]
    #req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    #response = db_service(req.return_cols,req.req_data)
    #print "Report "+response.report.data
    #for i in range(len(response.res_cols)):
      #sys.stdout.write(response.res_cols[i].data+" ")
    #sys.stdout.write('\n')
    #for i in range(len(response.res_data)):
      #for j in range(len(response.res_data[i].s)):
        #sys.stdout.write(response.res_data[i].s[j].data+" ")
      #sys.stdout.write('\n')
    #sys.stdout.write('\n')
    #self.assertEqual(response.report.data,"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_cols[0].data,"id")
    #self.assertEqual(response.res_cols[1].data,"username")
    #self.assertEqual(response.res_cols[2].data,"firstname")
    #self.assertEqual(response.res_cols[3].data,"lastname")
    #self.assertEqual(response.res_cols[4].data,"email_id")
    #self.assertEqual(response.res_cols[5].data,"pwd")
    #self.assertEqual(response.res_cols[6].data,"usrgroup")
    #self.assertEqual(response.res_cols[7].data,"created")
    #self.assertEqual(response.res_cols[8].data,"accessed")
    #self.assertEqual(response.res_cols[9].data,"enabled")
    #self.assertEqual(response.res_cols[10].data,"activation")
    #self.assertEqual(response.res_data[0].s[9].data,"1")
    #self.assertEqual(response.res_data[0].s[4].data,"0")
    #self.assertEqual(response.res_data[0].s[0].data,"1")

  #def testSubmitMoreLinesQuery(self):
    #serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("Speech detection topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    #req = DbWrapperSrv()
    #req.return_cols=[String(data="*")]
    #entry1=StringArrayMsg()
    #entry1=[String(data="firstname"), String(data="Alex")]
    #req.req_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.return_cols,req.req_data)
    #self.assertEqual(response.report.data,"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[0].data,"1")
    #self.assertEqual(response.res_data[0].s[4].data,"0")
    #self.assertEqual(response.res_data[1].s[0].data,"2")
    #self.assertEqual(response.res_data[1].s[5].data,"486d18ed96603f0bbae4567y2c98cc80750402b28c1d4069d5df7c570ded0307")


  ##test tblAppsRobots
  #def testTblAppsRobotsRead(self):
    ##Read
    #serv_topic = rospy.get_param('mysql_wrapper_apps_robots_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("mysql_wrapper_apps_robots_fetch_data_topic topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, fetchDataSrv)
    #req = fetchDataSrv()
    #req.req_cols=[String(data="app_id"),String(data="robot_id")]
    #entry1=[String(data="app_id"),String(data="1")]
    #req.where_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.req_cols,req.where_data)
    #self.assertEqual(response.trace[0].data,"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[0].data,"1")
    #self.assertEqual(response.res_data[0].s[1].data,"1")


  ##test tblUsersOntologyInstances
  #def testTblUsersOntologyInstancesRead(self):
    ##Read
    #serv_topic = rospy.get_param('mysql_wrapper_users_ontology_instances_fetch_data_topic')
    #if(not serv_topic):
      #rospy.logerror("Speech detection topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    #req = DbWrapperSrv()
    #req.return_cols=[String(data="id"),String(data="ontology_class"),String(data="ontology_instance")]
    #entry1=[String(data="id"),String(data="1")]
    #req.req_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.return_cols,req.req_data)
    #self.assertEqual(response.report.data,"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[1].data,"ontology_something")
    #self.assertEqual(response.res_data[0].s[2].data,"instance_something")

  ##test viewUsersRobotsApps
  #def testviewUsersRobotsAppsRead(self):
    ##Read
    #serv_topic = rospy.get_param('viewUsersRobotsApps_topic')
    #if(not serv_topic):
      #rospy.logerror("Speech detection topic param not found")
    #rospy.wait_for_service(serv_topic)
    #db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    #req = DbWrapperSrv()
    #req.return_cols=[String(data="owner"),String(data="id"),String(data="app_id")]
    #entry1=[String(data="id"),String(data="1")]
    #req.req_data=[StringArrayMsg(s=entry1)]
    #response = db_service(req.return_cols,req.req_data)
    #self.assertEqual(response.report.data,"Success")
    #self.assertTrue(response.success.data)
    #self.assertEqual(response.res_data[0].s[1].data,"1")
    #self.assertEqual(response.res_data[0].s[2].data,"1")
    #self.assertEqual(response.res_data[0].s[2].data,"1")
