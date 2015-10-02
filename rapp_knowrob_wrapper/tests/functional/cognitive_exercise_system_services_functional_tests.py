#!/usr/bin/env python

PKG='rapp_knowrob_wrapper'

import sys
import unittest
import rospy
import roslib

from rapp_platform_ros_communications.srv import (
  cognitiveTestsOfTypeSrv,
  cognitiveTestsOfTypeSrvRequest,
  cognitiveTestsOfTypeSrvResponse,
  createCognitiveExerciseTestSrv,
  createCognitiveExerciseTestSrvRequest,
  createCognitiveExerciseTestSrvResponse,
  recordUserPerformanceCognitiveTestsSrv,
  recordUserPerformanceCognitiveTestsSrvRequest,
  recordUserPerformanceCognitiveTestsSrvResponse,
  userPerformanceCognitveTestsSrv,
  userPerformanceCognitveTestsSrvRequest,
  userPerformanceCognitveTestsSrvResponse
  )

class OntologyFunc(unittest.TestCase):

    # Subclasses_of tests
    def test_cognitive_tests_of_existent_type(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_cognitive_tests_of_type")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, cognitiveTestsOfTypeSrv)

        req = cognitiveTestsOfTypeSrvRequest()        
        req.test_type="ArithmeticCts"
        response = test_service(req)     
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(prefix + 'ArithmeticCts_qdaDeDZn' in response.tests, True)
        self.assertEqual(prefix + 'ArithmeticCts_bneXbLGX' in response.tests, True)   
        self.assertEqual(response.success, True)
        
    def test_cognitive_tests_of_nonexistent_type(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_cognitive_tests_of_type")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, cognitiveTestsOfTypeSrv)

        req = cognitiveTestsOfTypeSrvRequest()        
        req.test_type="Kati"
        response = test_service(req)        
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'No tests of given type exist')
        
    def test_user_performance_existent_user_and_test(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_user_performance_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, userPerformanceCognitveTestsSrv)

        req = userPerformanceCognitveTestsSrvRequest()        
        req.ontology_alias="Person_vUXiHMJy"
        req.test_type="ArithmeticCts"
        response = test_service(req)        
        self.assertEqual(response.success, True)
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(prefix + 'ArithmeticCts_qdaDeDZn' in response.tests, True)
        self.assertEqual(prefix + 'ArithmeticCts_bneXbLGX' in response.tests, True) 
        
    def test_user_performance_nonexistent_user_and_test(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_user_performance_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, userPerformanceCognitveTestsSrv)

        req = userPerformanceCognitveTestsSrvRequest()        
        req.ontology_alias="11Person_vUXiHMJy"
        req.test_type="11ArithmeticCts"
        response = test_service(req)        
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'No performance records exist for the user or invalid user or invalid test type')
        
    def test_record_user_performance_existing_test_and_user(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_record_user_cognitive_tests_performance")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, recordUserPerformanceCognitiveTestsSrv)

        req = recordUserPerformanceCognitiveTestsSrvRequest()        
        req.patient_ontology_alias="Person_vUXiHMJy"
        req.test_type="ArithmeticCts"
        req.test="ArithmeticCts_qdaDeDZn"
        req.score=1
        req.timestamp=1
        response = test_service(req)        
        self.assertEqual(response.success, True)
        
    def test_record_user_performance_nonexisting_test_and_user(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_record_user_cognitive_tests_performance")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, recordUserPerformanceCognitiveTestsSrv)

        req = recordUserPerformanceCognitiveTestsSrvRequest()        
        req.patient_ontology_alias="1Person_vUXiHMJy"
        req.test_type="1ArithmeticCts"
        req.test="1Arithm1eticCts_qdaDeDZn"
        req.score=1
        req.timestamp=1
        response = test_service(req)        
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'Test performance entry insertion into ontology FAILED, either invalid test or patient alias')
        
    def test_create_existent_cognitive_test(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_create_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, createCognitiveExerciseTestSrv)

        req = createCognitiveExerciseTestSrvRequest()
        req.test_type="ArithmeticCts"
        req.test_difficulty=1
        req.test_variation=1
        req.test_path="/cognitiveTests/additionsTest1.xml"
        req.test_subtype="BasicArithmeticCts"
        response = test_service(req)        
        self.assertEqual(response.success, True)       

    def test_create_nonexistent_cognitive_test(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_create_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, createCognitiveExerciseTestSrv)

        req = createCognitiveExerciseTestSrvRequest()
        req.test_type="1ArithmeticCts"
        req.test_difficulty=1
        req.test_variation=1
        req.test_path="/cognitiveTests/additionsTest1.xml"
        req.test_subtype="BasicArithmeticCts"
        response = test_service(req)        
        self.assertEqual(response.success, False)  
        self.assertEqual(response.error, 'Test insertion into ontology FAILED, possible error is test type/subtype invalid')
        
    def test_create_nonexistent_cognitive_test_xml_path_nonexistent(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_create_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(\
                subclasses_of_service, createCognitiveExerciseTestSrv)

        req = createCognitiveExerciseTestSrvRequest()
        req.test_type="ArithmeticCts"
        req.test_difficulty=1
        req.test_variation=1
        req.test_path="1/1cognitiveTests/additionsTest1.xml"
        req.test_subtype="BasicArithmeticCts"
        response = test_service(req)        
        self.assertEqual(response.success, False)  
        self.assertEqual(response.error, 'Test file does not exist in provided file path')

    
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'OntologyFunc', OntologyFunc)














