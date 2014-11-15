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
from rapp_platform_ros_communications.srv import *

class TestDbWrapper(unittest.TestCase):      

  def testSubmitQuery(self):
    rospy.wait_for_service('ric/mysql_wrapper_service')
    db_service = rospy.ServiceProxy('ric/mysql_wrapper_service', DB)
    response = db_service("test_name", "1")
    s=response.sum
    #print "Error %s" % (s)
    self.assertEqual( s, "test_surname","Query returned correct answer")
    #if s=="ydraylikos":
    #    self.assertTrue(True,'ok')
    #else:
    #     self.assertTrue(False,'error, unexpected string')
          
if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'TestDbWrapper', TestDbWrapper)














