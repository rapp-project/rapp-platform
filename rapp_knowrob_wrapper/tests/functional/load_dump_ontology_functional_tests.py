#!/usr/bin/env python

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

class OntologyFunc(unittest.TestCase):

    # Subclasses_of tests
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
  
    
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'OntologyFunc', OntologyFunc)














