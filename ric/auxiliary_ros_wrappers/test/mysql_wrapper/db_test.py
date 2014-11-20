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

#class TestDbWrapper(unittest.TestCase):      

  #def testSubmitQuery(self):
rospy.wait_for_service('ric/mysql_wrapper_service')
db_service = rospy.ServiceProxy('ric/mysql_wrapper_service', DbWrapperSrv)

req = DbWrapperSrv()
req.return_cols=[String(data="id"), String(data="username"), String(data="firstname")]

###
t1=StringArrayMsg() 
t1=[]
t1=t1+[String(data="ttname")]
t1=t1+[String(data="ttname")]
t1=t1+[String(data="t`11tname")]
print t1[0].data

###
print "exit test"
sam=StringArrayMsg()    #=[String(data="1"), String(data="2"), String(data="3")]
sam=[String(data="username"), String(data="admin")]
sam1=StringArrayMsg()
sam1=[String(data="firstname"), String(data="Alex")]
#print sam[0].data

req.req_data=[StringArrayMsg(s=sam),StringArrayMsg(s=sam1)]

print req.req_data[1].s[1].data


response = db_service(req.return_cols,req.req_data)



#print starray[0].data
#print req_data.data[0].s[0]
#response = db_service(req)
    
    
    
    #s=response.sum
    #m=response.success.data
    #print "Error %s" % (s)
    #self.assertEqual( s, "test_surname","Query returned correct answer")
    #self.assertTrue(response.success)
    
    
    
    #self.assertEqual("2",(response.results[1].data)) #works
    
    #if s=="ydraylikos":
    #    self.assertTrue(True,'ok')
    #else:
    #     self.assertTrue(False,'error, unexpected string')
          
#if __name__ == '__main__':
#  import rosunit
 # rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)














