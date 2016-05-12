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
  userPerformanceCognitveTestsSrvResponse,
  createOntologyAliasSrv,
  createOntologyAliasSrvRequest,
  createOntologyAliasSrvResponse
  )

## @class OntologyFunc
# Inherits the unittest.TestCase class in order to offer functional tests functionality
class OntologyFunc(unittest.TestCase):

    ## Test cognitive tests of existent type
    def test_cognitive_tests_of_existent_type(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_cognitive_tests_of_type")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, cognitiveTestsOfTypeSrv)

        req = cognitiveTestsOfTypeSrvRequest()
        req.test_type="ArithmeticCts"
        req.test_language="el"
        response = test_service(req)
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(prefix + 'ArithmeticCts_tDjYwuhx' in response.tests, True)
        self.assertEqual(prefix + 'ArithmeticCts_bneXbLGX' in response.tests, True)
        self.assertEqual(response.success, True)

	## Test cognitive tests of non existent type
    def test_cognitive_tests_of_nonexistent_type(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_cognitive_tests_of_type")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, cognitiveTestsOfTypeSrv)

        req = cognitiveTestsOfTypeSrvRequest()
        req.test_type="Kati"
        req.test_language="el"
        response = test_service(req)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'No tests of given type exist')

    ## Test get user ontology alias of existent user
    def test_get_user_ontology_alias_existent_user(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_create_ontology_alias")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, createOntologyAliasSrv)

        req = createOntologyAliasSrvRequest()
        req.username="rapp"
        response = test_service(req)
        self.assertEqual(response.success, True)
        self.assertEqual(response.ontology_alias,'Person_DpphmPqg')

    ## Test get user ontology alias of nonexistent user
    def test_get_user_ontology_alias_nonexistent_user(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_create_ontology_alias")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, createOntologyAliasSrv)

        req = createOntologyAliasSrvRequest()
        req.username="rapprapprapprapp"
        response = test_service(req)
        self.assertEqual(response.success, False)

	## Test user performance of existent user and test type
    def test_user_performance_existent_user_and_test(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_user_performance_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, userPerformanceCognitveTestsSrv)

        req = userPerformanceCognitveTestsSrvRequest()
        req.ontology_alias="Person_DpphmPqg"
        req.test_type="ArithmeticCts"
        response = test_service(req)
        self.assertEqual(response.success, True)
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(prefix + 'ArithmeticCts_bneXbLGX' in response.tests, True)

	## Test user performance of nonexistent user and test type
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

	## Test record user performance of existent user and test type
    #def test_record_user_performance_existing_test_and_user(self):
        #subclasses_of_service = rospy.get_param(\
                #"rapp_knowrob_wrapper_record_user_cognitive_tests_performance")
        #rospy.wait_for_service(subclasses_of_service)

        #test_service = rospy.ServiceProxy(\
                #subclasses_of_service, recordUserPerformanceCognitiveTestsSrv)

        #req = recordUserPerformanceCognitiveTestsSrvRequest()
        #req.patient_ontology_alias="Person_DpphmPqg"
        ##req.test_type="ArithmeticCts"
        #req.test="ArithmeticCts_MewmmEsP"
        #req.score=11
        #req.timestamp=1
        #response = test_service(req)
        #self.assertEqual(response.success, True)

	## Test record user performance of nonexistent user and test type
    def test_record_user_performance_nonexisting_test_and_user(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_record_user_cognitive_tests_performance")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, recordUserPerformanceCognitiveTestsSrv)

        req = recordUserPerformanceCognitiveTestsSrvRequest()
        req.patient_ontology_alias="1Person_vUXiHMJy"
        req.test="1Arithm1eticCts_qdaDeDZn"
        req.score=1
        req.timestamp=1
        response = test_service(req)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'Test performance entry insertion into ontology FAILED, either invalid test or patient alias')

	## Test create existent cognitive test
    #def test_create_existent_cognitive_test(self):
        #subclasses_of_service = rospy.get_param(\
                #"rapp_knowrob_wrapper_create_cognitve_tests")
        #rospy.wait_for_service(subclasses_of_service)

        #test_service = rospy.ServiceProxy(\
                #subclasses_of_service, createCognitiveExerciseTestSrv)

        #req = createCognitiveExerciseTestSrvRequest()
        #req.test_type="ArithmeticCts"
        #req.test_difficulty=1
        #req.test_path="/cognitiveTests/ArithmeticCts_BasicArithmeticCts_diff1__var1.xml"
        #req.test_subtype="BasicArithmeticCts"
        #response = test_service(req)
        #self.assertEqual(response.success, True)

	## Test create non existent cognitive test
    def test_create_nonexistent_cognitive_test(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_create_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, createCognitiveExerciseTestSrv)

        req = createCognitiveExerciseTestSrvRequest()
        req.test_type="1ArithmeticCts"
        req.test_difficulty=1
        req.test_path="/cognitiveTests/ArithmeticCts_BasicArithmeticCts_diff1_id_0.xml"
        req.test_subtype="BasicArithmeticCts"
        supportedLanguages=['en','el']
        req.supported_languages=supportedLanguages
        response = test_service(req)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'Test insertion into ontology FAILED, possible error is test type/subtype invalid')

	## Test create non existent cognitive test with invalid xml path
    def test_create_nonexistent_cognitive_test_xml_path_nonexistent(self):
        subclasses_of_service = rospy.get_param(\
                "rapp_knowrob_wrapper_create_cognitve_tests")
        rospy.wait_for_service(subclasses_of_service)

        test_service = rospy.ServiceProxy(\
                subclasses_of_service, createCognitiveExerciseTestSrv)

        req = createCognitiveExerciseTestSrvRequest()
        req.test_type="ArithmeticCts"
        req.test_difficulty=1
        req.test_path="1/1cognitiveTests/additionsTest1.xml"
        req.test_subtype="BasicArithmeticCts"
        response = test_service(req)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'Test file does not exist in provided file path')

## The main function. Initializes the Cognitive Exercise System functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'OntologyFunc', OntologyFunc)
