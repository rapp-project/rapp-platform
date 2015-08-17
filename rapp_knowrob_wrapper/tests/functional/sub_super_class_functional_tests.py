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

    # Subclasses_of tests
    def test_subclasses_of_existent_class(self):
        subclasses_of_service = rospy.get_param("rapp_knowrob_wrapper_subclasses_of_topic")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(subclasses_of_service, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'Oven'
        response = test_service(req)
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(prefix + 'RegularOven' in response.results, True)
        self.assertEqual(prefix + 'MicrowaveOven' in response.results, True)
        self.assertEqual(prefix + 'ToasterOven' in response.results, True)

    def test_subclasses_of_non_existent_class(self):
        subclasses_of_service = rospy.get_param("rapp_knowrob_wrapper_subclasses_of_topic")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(subclasses_of_service, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'Podosfairo'
        response = test_service(req)
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(response.results, [])
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'Class: Podosfairo does not exist')

    def test_subclasses_of_existent_class_recursive(self):
        subclasses_of_service = rospy.get_param("rapp_knowrob_wrapper_subclasses_of_topic")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(subclasses_of_service, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'Oven'
        req.recursive = True
        response = test_service(req)
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(prefix + 'RegularOven' in response.results, True)
        self.assertEqual(prefix + 'Oven' in response.results, True)
        self.assertEqual(prefix + 'MicrowaveOven' in response.results, True)
        self.assertEqual(prefix + 'ToasterOven' in response.results, True)
        self.assertEqual(response.success, True)
        self.assertEqual(response.error, "")

    def test_subclasses_of_non_existent_class_recursive(self):
        subclasses_of_service = rospy.get_param("rapp_knowrob_wrapper_subclasses_of_topic")
        rospy.wait_for_service(subclasses_of_service)
        
        test_service = rospy.ServiceProxy(subclasses_of_service, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'Podosfairo'
        req.recursive = True
        response = test_service(req)
        prefix = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(response.results, [])
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, 'Class: Podosfairo does not exist')

    # Superclasses_of tests
    def test_superclasses_of_existent_class(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_superclasses_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'SpatialThing'
        response = test_service(req)
        prefix = 'http://www.w3.org/2002/07/owl#'
        self.assertEqual(prefix + 'Thing' in response.results, True)

    def test_superclasses_of_existent_class_recursive(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_superclasses_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'Oven'
        req.recursive = True
        response = test_service(req)
        prefix = 'http://www.w3.org/2002/07/owl#'
        prefix_2 = 'http://knowrob.org/kb/knowrob.owl#'
        self.assertEqual(len(response.results), 20)
        self.assertEqual(prefix + 'Thing' in response.results, True)
        self.assertEqual(prefix_2 + 'SpatialThing' in response.results, True)
        self.assertEqual(prefix_2 + 'HouseholdAppliance' in response.results, True)

    def test_superclasses_of_non_existent_class(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_superclasses_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'NotExistent'
        response = test_service(req)
        self.assertEqual(response.results, [])
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, "Class: NotExistent does not exist")

    def test_superclasses_of_non_existent_class_recursive(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_superclasses_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologySubSuperClassesOfSrv)

        req = ontologySubSuperClassesOfSrvRequest()
        req.ontology_class = 'NotExistent'
        req.recursive = True
        response = test_service(req)
        self.assertEqual(response.results, [])
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, "Class: NotExistent does not exist")

    # Is sub-superclasses_of tests
    def test_sub_superclasses_of_existent_class_direct(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'Oven'
        req.child_class = 'MicrowaveOven'
        req.recursive = False

        response = test_service(req)
        self.assertEqual(response.result, True)

    def test_sub_superclasses_of_existent_class_indirect(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'SpatialThing'
        req.child_class = 'MicrowaveOven'
        req.recursive = False

        response = test_service(req)
        self.assertEqual(response.result, False)

    def test_sub_superclasses_of_existent_class_direct_recursive(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'Oven'
        req.child_class = 'MicrowaveOven'
        req.recursive = True

        response = test_service(req)
        self.assertEqual(response.result, True)

    def test_sub_superclasses_of_existent_class_indirect_recursive(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'SpatialThing'
        req.child_class = 'MicrowaveOven'
        req.recursive = True

        response = test_service(req)
        self.assertEqual(response.result, True)

    def test_sub_superclasses_of_non_existent_class(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'Aoua'
        req.child_class = 'MicrowaveOven'
        req.recursive = False

        response = test_service(req)
        self.assertEqual(response.result, False)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, "Class: Aoua does not exist")

    def test_sub_superclasses_of_existent_class_recursive(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'Aoua'
        req.child_class = 'MicrowaveOven'
        req.recursive = True

        response = test_service(req)
        self.assertEqual(response.result, False)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, "Class: Aoua does not exist")

    def test_sub_superclasses_of_non_existent_classes(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'Aoua'
        req.child_class = 'Aoua2'
        req.recursive = False

        response = test_service(req)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, "Class: Aoua does not exist")
        self.assertEqual(response.result, False)

    def test_sub_superclasses_of_existent_classes_recursive(self):
        service_name = rospy.get_param("rapp_knowrob_wrapper_is_subsuperclass_of_topic")
        rospy.wait_for_service(service_name)
        
        test_service = rospy.ServiceProxy(service_name, ontologyIsSubSuperClassOfSrv)

        req = ontologyIsSubSuperClassOfSrvRequest()
        req.parent_class = 'Aoua'
        req.child_class = 'Aoua2'
        req.recursive = True

        response = test_service(req)
        self.assertEqual(response.result, False)
        self.assertEqual(response.success, False)
        self.assertEqual(response.error, "Class: Aoua does not exist")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'OntologyFunc', OntologyFunc)














