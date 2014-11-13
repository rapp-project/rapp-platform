#!/usr/bin/env python
PKG='auxiliary_ros_wrappers'
#import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest
import rospy
from rapp_platform_ros_communications.srv import *


## A sample python unit test
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














