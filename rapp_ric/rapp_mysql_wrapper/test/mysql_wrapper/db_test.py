#!/usr/bin/env python

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr

PKG='test_rapp_mysql_wrapper'
import sys
import unittest
import rospy

from rapp_platform_ros_communications.srv import (
  DbWrapperSrv,
  DbWrapperSrvResponse
  )

from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  ) 


from std_msgs.msg import ( 
  String
  ) 

class TestDbWrapper(unittest.TestCase):      

  def testSubmitCorrectQuery(self):
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="id"), String(data="username"), String(data="firstname"), String(data="email_id")]
    #req.return_cols=[String(data="*")]
    entry1=StringArrayMsg()    
    entry1=[String(data="username"), String(data="admin")]
    entry2=StringArrayMsg()
    entry2=[String(data="firstname"), String(data="Alex")]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]

    response = db_service(req.return_cols,req.req_data)
    print "Report "+response.report.data
    for i in range(len(response.res_cols)):
      sys.stdout.write(response.res_cols[i].data+" ")      
    sys.stdout.write('\n')
    for i in range(len(response.res_data)):
      for j in range(len(response.res_data[i].s)):
        sys.stdout.write(response.res_data[i].s[j].data+" ")
      sys.stdout.write('\n')
    sys.stdout.write('\n')
    self.assertTrue(response.success.data)
    self.assertEqual(response.report.data,"Success")
    self.assertEqual(response.res_cols[0].data,"id")
    self.assertEqual(response.res_cols[1].data,"username")
    self.assertEqual(response.res_cols[2].data,"firstname")
    self.assertEqual(response.res_cols[3].data,"email_id")
    self.assertEqual(response.res_data[0].s[0].data,"1")
    self.assertEqual(response.res_data[0].s[1].data,"admin")
    self.assertEqual(response.res_data[0].s[3].data,"0")
  
  def testSubmitIncompleteQuery(self):
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]#[String(data="id"), String(data="username"), String(data="firstname"), String(data="otinanai")]
    entry1=StringArrayMsg() 
    #entry1=[]
    entry1=[String(data="username")]       
    entry2=StringArrayMsg()    
    entry2=[] 
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    response = db_service(req.return_cols,req.req_data)
    self.assertFalse(response.success.data)
    self.assertEqual(response.report.data,"Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format")

  def testSubmitWrongQuery(self):
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="id"), String(data="username"), String(data="firstname"), String(data="otinanai")]    
    entry1=StringArrayMsg()    
    entry1=[String(data="username"), String(data="admin")]
    entry2=StringArrayMsg()
    entry2=[String(data="firstname"), String(data="Alex")]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    response = db_service(req.return_cols,req.req_data)
    self.assertFalse(response.success.data)
    errorStart=response.report.data[0:14]
    print errorStart
    self.assertEqual(errorStart,"Database Error")
    
  def testSubmitCorrectReturnAllColumnQuery(self):
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()    
    req.return_cols=[String(data="*")]
    entry1=StringArrayMsg()    
    entry1=[String(data="username"), String(data="admin")]
    entry2=StringArrayMsg()
    entry2=[String(data="firstname"), String(data="Alex")]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    response = db_service(req.return_cols,req.req_data)
    print "Report "+response.report.data
    for i in range(len(response.res_cols)):
      sys.stdout.write(response.res_cols[i].data+" ")      
    sys.stdout.write('\n')
    for i in range(len(response.res_data)):
      for j in range(len(response.res_data[i].s)):
        sys.stdout.write(response.res_data[i].s[j].data+" ")
      sys.stdout.write('\n')
    sys.stdout.write('\n')
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_cols[0].data,"id")
    self.assertEqual(response.res_cols[1].data,"username")
    self.assertEqual(response.res_cols[2].data,"firstname")
    self.assertEqual(response.res_cols[3].data,"lastname")
    self.assertEqual(response.res_cols[4].data,"email_id")
    self.assertEqual(response.res_cols[5].data,"pwd")
    self.assertEqual(response.res_cols[6].data,"usrgroup")
    self.assertEqual(response.res_cols[7].data,"created")
    self.assertEqual(response.res_cols[8].data,"accessed")
    self.assertEqual(response.res_cols[9].data,"enabled")
    self.assertEqual(response.res_cols[10].data,"activation")    
    self.assertEqual(response.res_data[0].s[9].data,"1")
    self.assertEqual(response.res_data[0].s[4].data,"0")
    self.assertEqual(response.res_data[0].s[0].data,"1")
    
  def testSubmitMoreLinesQuery(self):
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()    
    req.return_cols=[String(data="*")]
    entry1=StringArrayMsg()    
    entry1=[String(data="firstname"), String(data="Alex")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0].data,"1")
    self.assertEqual(response.res_data[0].s[4].data,"0")
    self.assertEqual(response.res_data[1].s[0].data,"2")
    self.assertEqual(response.res_data[1].s[5].data,"486d18ed96603f0bbae4567y2c98cc80750402b28c1d4069d5df7c570ded0307")
    
  #test TblUser
  def testTblUserWriteReadDeleteCheck(self):
    #Write
    serv_topic = rospy.get_param('mysql_wrapper_user_write_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]#[String(data="model_str"),String(data="manufacturer"),String(data="version"),String(data="arch"),String(data="os"),String(data="picture")]
    req.return_cols=[String(data="id"),String(data="username"), String(data="firstname"),String(data="lastname"), String(data="email_id"),String(data="pwd"), String(data="usrgroup"),String(data="created"), String(data="accessed"),String(data="enabled"), String(data="activation")]
    entry1=StringArrayMsg()    
    entry1=[String(data="'11'"),String(data="'merk2'"), String(data="'Alex2'"),String(data="'Marko2'"), String(data="'1'"),String(data="'86'"), String(data="'0'"),String(data="'2014-15-15 18:01:34'"), String(data="'0000-00-00 00:00:00'"),String(data="'1'"), String(data="'555'")]
    entry2=StringArrayMsg()    
    entry2=[String(data="'12'"),String(data="'merk3'"), String(data="'Alex2'"),String(data="'Marko3'"), String(data="'1'"),String(data="'86'"), String(data="'0'"),String(data="'2014-15-15 18:01:34'"), String(data="'0000-00-00 00:00:00'"),String(data="'1'"), String(data="'555'")]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    #Read what was written
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="firstname"),String(data="lastname")]
    entry1=[String(data="firstname"),String(data="Alex2")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    self.assertEqual(response.res_data[0].s[1].data,"Marko2")
    self.assertEqual(response.res_data[1].s[1].data,"Marko3")    
    #Update written
    serv_topic = rospy.get_param('mysql_wrapper_user_update_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="firstname='Alex3'")]
    entry1=[String(data="firstname"),String(data='Alex2')]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)       
    #Read again
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="firstname"),String(data="lastname")]
    entry1=[String(data="firstname"),String(data="Alex3")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0].data,"Alex3")
    self.assertEqual(response.res_data[1].s[0].data,"Alex3")    
    #Delete updated
    serv_topic = rospy.get_param('mysql_wrapper_user_delete_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]
    entry1=[String(data="firstname"),String(data="Alex3")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)   
    #Check if it was deleted
    serv_topic = rospy.get_param('mysql_wrapper_user_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="firstname"),String(data="lastname")]
    entry1=[String(data="firstname"),String(data="Alex3")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)    
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertTrue((len(response.res_data)<1))
    
  #test tblModel  
  def testTblModelWriteReadDeleteCheck(self):
    #Write
    serv_topic = rospy.get_param('mysql_wrapper_model_write_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]#[String(data="model_str"),String(data="manufacturer"),String(data="version"),String(data="arch"),String(data="os"),String(data="picture")]
    req.return_cols=[String(data="id"),String(data="model_str"), String(data="manufacturer"),String(data="version"), String(data="arch"),String(data="os"), String(data="picture")]
    entry1=StringArrayMsg()    
    entry1=[String(data="'2'"),String(data="'extreme2'"), String(data="'antaIndustries'"),String(data="'1.100000000'"), String(data="'1.5'"),String(data="'ubuntu 14.04'"), String(data="'noth1'")]
    entry2=StringArrayMsg()    
    entry2=[String(data="'3'"),String(data="'extreme3'"), String(data="'antaIndustries'"),String(data="'1.110000000'"), String(data="'1.6'"),String(data="'ubuntu 14.04'"), String(data="'roth2'")]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    #Read what was written
    serv_topic = rospy.get_param('mysql_wrapper_model_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="model_str"),String(data="manufacturer")]
    entry1=[String(data="manufacturer"),String(data="antaIndustries")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    self.assertEqual(response.res_data[0].s[0].data,"extreme2")
    self.assertEqual(response.res_data[1].s[0].data,"extreme3")    
    #Update written
    serv_topic = rospy.get_param('mysql_wrapper_model_update_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="model_str='extreme3'")]
    entry1=[String(data="model_str"),String(data='extreme2')]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)       
    #Read again
    serv_topic = rospy.get_param('mysql_wrapper_model_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="model_str"),String(data="manufacturer")]
    entry1=[String(data="model_str"),String(data="extreme3")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0].data,"extreme3")
    self.assertEqual(response.res_data[1].s[0].data,"extreme3")    
    #Delete updated
    serv_topic = rospy.get_param('mysql_wrapper_model_delete_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]
    entry1=[String(data="model_str"),String(data="extreme3")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)   
    #Check if it was deleted
    serv_topic = rospy.get_param('mysql_wrapper_model_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="model_str"),String(data="manufacturer")]
    entry1=[String(data="model_str"),String(data="extreme3")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)    
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertTrue((len(response.res_data)<1))
    
  #test tbRapp  
  def testTblRappWriteReadDeleteCheck(self):
    #Write
    serv_topic = rospy.get_param('mysql_wrapper_rapp_write_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]#[String(data="model_str"),String(data="manufacturer"),String(data="version"),String(data="arch"),String(data="os"),String(data="picture")]
    req.return_cols=[String(data="id"),String(data="rapp"), String(data="version"),String(data="arch"), String(data="owner"),String(data="directory"), String(data="enabled"),String(data="timestamp")]
    entry1=StringArrayMsg()    
    entry1=[String(data="'2'"),String(data="'uberApp4'"), String(data="'1.910000000'"),String(data="'15'"), String(data="'1'"),String(data="'homethanos'"), String(data="'0'"),String(data="'2014-11-23 09:04:13'")]
    entry2=StringArrayMsg()    
    entry2=[String(data="'3'"),String(data="'uberApp2'"), String(data="'1.810000000'"),String(data="'16'"), String(data="'1'"),String(data="'homethanos'"), String(data="'0'"),String(data="'2014-11-23 07:04:13'")]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    #Read what was written
    serv_topic = rospy.get_param('mysql_wrapper_rapp_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="rapp"),String(data="version")]
    entry1=[String(data="rapp"),String(data="uberApp1")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    self.assertEqual(response.res_data[0].s[0].data,"uberApp1")
    #self.assertEqual(response.res_data[1].s[0].data,"extreme3")    
    #Update written
    serv_topic = rospy.get_param('mysql_wrapper_rapp_update_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="rapp='uberApp2'")]
    entry1=[String(data="rapp"),String(data='uberApp4')]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)       
    #Read again
    serv_topic = rospy.get_param('mysql_wrapper_rapp_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="rapp"),String(data="version")]
    entry1=[String(data="rapp"),String(data="uberApp2")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0].data,"uberApp2")
    self.assertEqual(response.res_data[1].s[0].data,"uberApp2")    
    #Delete updated
    serv_topic = rospy.get_param('mysql_wrapper_rapp_delete_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]
    entry1=[String(data="rapp"),String(data="uberApp2")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)   
    #Check if it was deleted
    serv_topic = rospy.get_param('mysql_wrapper_rapp_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="rapp"),String(data="version")]
    entry1=[String(data="rapp"),String(data="uberApp2")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)    
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertTrue((len(response.res_data)<1))

  #test tbRobot
  def testTblRobotWriteReadDeleteCheck(self):
    #Write
    serv_topic = rospy.get_param('mysql_wrapper_robot_write_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]#[String(data="model_str"),String(data="manufacturer"),String(data="version"),String(data="arch"),String(data="os"),String(data="picture")]
    req.return_cols=[String(data="id"),String(data="macddr"), String(data="model"),String(data="owner"), String(data="timestamp")]
    entry1=StringArrayMsg()    
    entry1=[String(data="'3'"),String(data="'1800000'"), String(data="'1'"),String(data="'1'"), String(data="'2014-11-23 09:04:13'")]
    entry2=StringArrayMsg()    
    entry2=[String(data="'4'"),String(data="'1900000'"), String(data="'1'"),String(data="'1'"), String(data="'2014-11-23 07:04:13'")]
    req.req_data=[StringArrayMsg(s=entry1),StringArrayMsg(s=entry2)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    #Read what was written
    serv_topic = rospy.get_param('mysql_wrapper_robot_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="id"),String(data="macddr")]
    entry1=[String(data="macddr"),String(data="1800000")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    self.assertEqual(response.res_data[0].s[0].data,"3")
    #self.assertEqual(response.res_data[1].s[0].data,"extreme3")    
    #Update written
    serv_topic = rospy.get_param('mysql_wrapper_robot_update_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="timestamp='2014-11-23 09:04:13'")]
    entry1=[String(data="macddr"),String(data='1900000')]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)       
    #Read again
    serv_topic = rospy.get_param('mysql_wrapper_robot_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="macddr"),String(data="model")]
    entry1=[String(data="timestamp"),String(data="2014-11-23 09:04:13")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0].data,"1800000")
    self.assertEqual(response.res_data[1].s[0].data,"1900000")    
    #Delete updated
    serv_topic = rospy.get_param('mysql_wrapper_robot_delete_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[]
    entry1=[String(data="timestamp"),String(data="2014-11-23 09:04:13")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)   
    #Check if it was deleted
    serv_topic = rospy.get_param('mysql_wrapper_robot_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="macddr"),String(data="model")]
    entry1=[String(data="timestamp"),String(data="2014-11-23 09:04:13")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)    
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)
    self.assertTrue((len(response.res_data)<1))
    
    
  #test tblAppsRobots
  def testTblAppsRobotsRead(self):
    #Read
    serv_topic = rospy.get_param('mysql_wrapper_apps_robots_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="app_id"),String(data="robot_id")]
    entry1=[String(data="app_id"),String(data="1")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    self.assertEqual(response.res_data[0].s[0].data,"1")
    self.assertEqual(response.res_data[0].s[1].data,"1")  
    
    
  #test tblUsersOntologyInstances
  def testTblUsersOntologyInstancesRead(self):
    #Read
    serv_topic = rospy.get_param('mysql_wrapper_users_ontology_instances_fetch_data_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="id"),String(data="ontology_class"),String(data="ontology_instance")]
    entry1=[String(data="id"),String(data="1")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    self.assertEqual(response.res_data[0].s[1].data,"ontology_something")
    self.assertEqual(response.res_data[0].s[2].data,"instance_something")  
    
  #test viewUsersRobotsApps
  def testviewUsersRobotsAppsRead(self):
    #Read
    serv_topic = rospy.get_param('viewUsersRobotsApps_topic')
    if(not serv_topic):
      rospy.logerror("Speech detection topic param not found")
    rospy.wait_for_service(serv_topic)
    db_service = rospy.ServiceProxy(serv_topic, DbWrapperSrv)
    req = DbWrapperSrv()
    req.return_cols=[String(data="owner"),String(data="id"),String(data="app_id")]
    entry1=[String(data="id"),String(data="1")]
    req.req_data=[StringArrayMsg(s=entry1)]
    response = db_service(req.return_cols,req.req_data)
    self.assertEqual(response.report.data,"Success")
    self.assertTrue(response.success.data)    
    self.assertEqual(response.res_data[0].s[1].data,"1")
    self.assertEqual(response.res_data[0].s[2].data,"1")  
    self.assertEqual(response.res_data[0].s[2].data,"1") 


if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)
