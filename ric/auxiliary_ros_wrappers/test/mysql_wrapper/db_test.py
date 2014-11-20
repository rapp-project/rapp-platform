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

PKG='auxiliary_ros_wrappers'
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
    rospy.wait_for_service('ric/db/mysql_wrapper_service')
    db_service = rospy.ServiceProxy('ric/db/mysql_wrapper_service', DbWrapperSrv)

    req = DbWrapperSrv()
    req.return_cols=[String(data="id"), String(data="username"), String(data="firstname"), String(data="email")]
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
    self.assertEqual(response.res_cols[3].data,"email")
    self.assertEqual(response.res_data[0].s[0].data,"1")
    self.assertEqual(response.res_data[0].s[1].data,"admin")
    self.assertEqual(response.res_data[0].s[3].data,"a.gkiokas@ortelio.co.uk")
  
  def testSubmitIncompleteQuery(self):
    rospy.wait_for_service('ric/db/mysql_wrapper_service')
    db_service = rospy.ServiceProxy('ric/db/mysql_wrapper_service', DbWrapperSrv)

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
    rospy.wait_for_service('ric/db/mysql_wrapper_service')
    db_service = rospy.ServiceProxy('ric/db/mysql_wrapper_service', DbWrapperSrv)
    
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
    rospy.wait_for_service('ric/db/mysql_wrapper_service')
    db_service = rospy.ServiceProxy('ric/db/mysql_wrapper_service', DbWrapperSrv)

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
    self.assertTrue(response.success.data)
    self.assertEqual(response.report.data,"Success")
    self.assertEqual(response.res_cols[0].data,"id")
    self.assertEqual(response.res_cols[1].data,"username")
    self.assertEqual(response.res_cols[2].data,"firstname")
    self.assertEqual(response.res_cols[3].data,"lastname")
    self.assertEqual(response.res_cols[4].data,"email")
    self.assertEqual(response.res_cols[5].data,"pwd")
    self.assertEqual(response.res_cols[6].data,"usrgroup")
    self.assertEqual(response.res_cols[7].data,"created")
    self.assertEqual(response.res_cols[8].data,"accessed")
    self.assertEqual(response.res_cols[9].data,"enabled")
    self.assertEqual(response.res_cols[10].data,"activation")
    
    self.assertEqual(response.res_data[0].s[9].data,"1")
    self.assertEqual(response.res_data[0].s[4].data,"a.gkiokas@ortelio.co.uk")
    self.assertEqual(response.res_data[0].s[0].data,"1")
    
  def testSubmitMoreLinesQuery(self):
    rospy.wait_for_service('ric/db/mysql_wrapper_service')
    db_service = rospy.ServiceProxy('ric/db/mysql_wrapper_service', DbWrapperSrv)

    req = DbWrapperSrv()    
    req.return_cols=[String(data="*")]

    entry1=StringArrayMsg()    
    entry1=[String(data="firstname"), String(data="Alex")]
    req.req_data=[StringArrayMsg(s=entry1)]

    response = db_service(req.return_cols,req.req_data)
    self.assertTrue(response.success.data)
    self.assertEqual(response.res_data[0].s[0].data,"1")
    self.assertEqual(response.res_data[0].s[4].data,"a.gkiokas@ortelio.co.uk")
    self.assertEqual(response.res_data[1].s[0].data,"2")
    self.assertEqual(response.res_data[1].s[5].data,"486d18ed96603f0bbae4567y2c98cc80750402b28c1d4069d5df7c570ded0307")

          
if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)














