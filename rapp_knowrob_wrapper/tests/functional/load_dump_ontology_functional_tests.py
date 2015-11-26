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


PKG='rapp_knowrob_wrapper'

import sys
import unittest
import rospy
import roslib

from rapp_platform_ros_communications.srv import (
  ontologyLoadDumpSrv,
  ontologyLoadDumpSrvRequest,
  ontologyLoadDumpSrvResponse
  )

## @class OntologyFunc 
# Inherits the unittest.TestCase class in order to offer functional tests functionalit
class OntologyFunc(unittest.TestCase):

    ## Test load ontology from existent file
    def test_load_existent(self):
        service = rospy.get_param(\
                "rapp_knowrob_wrapper_load_ontology_topic")
        rospy.wait_for_service(service)

        test_service = rospy.ServiceProxy(\
                service, ontologyLoadDumpSrv)

        req = ontologyLoadDumpSrvRequest()
        req.file_url="currentOntologyVersion.owl"
        response = test_service(req)
        self.assertEqual(response.success, True)
        
	## Test load ontology from non existent file
    def test_load_nonexistent(self):
        service = rospy.get_param(\
                "rapp_knowrob_wrapper_load_ontology_topic")
        rospy.wait_for_service(service)

        test_service = rospy.ServiceProxy(\
                service, ontologyLoadDumpSrv)

        req = ontologyLoadDumpSrvRequest()
        req.file_url="11currentOntologyVersion.owl"
        response = test_service(req)
        self.assertEqual(response.success, False)

	## Test dump ontology with correct path
    def test_dump_correct_path(self):
        service = rospy.get_param(\
                "rapp_knowrob_wrapper_dump_ontology_topic")
        rospy.wait_for_service(service)

        test_service = rospy.ServiceProxy(\
                service, ontologyLoadDumpSrv)

        req = ontologyLoadDumpSrvRequest()
        req.file_url="testDump.owl"
        response = test_service(req)
        self.assertEqual(response.success, True)

	## Test dump ontology with invalid path
    def test_dump_invalid_path(self):
        service = rospy.get_param(\
                "rapp_knowrob_wrapper_dump_ontology_topic")
        rospy.wait_for_service(service)

        test_service = rospy.ServiceProxy(\
                service, ontologyLoadDumpSrv)

        req = ontologyLoadDumpSrvRequest()
        req.file_url="/someInvalidPath/testDump.owl"
        response = test_service(req)
        self.assertEqual(response.success, False)

## The main function. Initializes the Cognitive Exercise System functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'OntologyFunc', OntologyFunc)














