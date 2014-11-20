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

  def testSubmitQuery(self):
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

    self.assertEqual(response.report.data,"Operation Successful")
    self.assertEqual(response.res_cols[0].data,"id")
    self.assertEqual(response.res_cols[1].data,"username")
    self.assertEqual(response.res_cols[2].data,"firstname")
    self.assertEqual(response.res_cols[3].data,"email")

          
if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)














