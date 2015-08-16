#!/usr/bin/env python

PKG='rapp_knowrob_wrapper'

import sys
import unittest
import rospy
import roslib

from rapp_platform_ros_communications.srv import (
  ontologyInstancesOfSrv,
  ontologyInstancesOfSrvRequest,
  ontologyInstancesOfSrvResponse,
  ontologySubSuperClassesOfSrv,
  ontologySubSuperClassesOfSrvRequest,
  ontologySubSuperClassesOfSrvResponse,
  ontologyIsSubSuperClassOfSrv,
  ontologyIsSubSuperClassOfSrvRequest,
  ontologyIsSubSuperClassOfSrvResponse
  )

class OntologyFunc(unittest.TestCase):

    def test_1(self):
        # subclasses_of_service = rospy.get_param("rapp_knowrob_wrapper_subclass_of_topic")
        # rospy.wait_for_service(subclasses_of_service)
        
        # test_service = rospy.ServiceProxy(subclasses_of_service, ontologySubSuperClassesOfSrv)

        # req = ontologySubSuperClassesOfSrvRequest()

        # req.ontology_class = 'Oven'
        # response = test_service(req)
        self.assertEqual( response.results, 1 )

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'OntologyFunc', OntologyFunc)














