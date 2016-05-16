#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr

import rospy
import sys

from image_classification import ImageClassification
from ontology_class_bridge import OntologyClassBridge
from image_registration_to_ontology import ImageRegistrationToOntology

from rapp_platform_ros_communications.srv import (
  imageClassificationSrv,
  imageClassificationSrvResponse,
  ontologyClassBridgeSrv,
  ontologyClassBridgeSrvResponse,
  registerImageToOntologySrv,
  registerImageToOntologySrvResponse 
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from std_msgs.msg import (
  String
  )

## @class RappCaffeWrapper
# @brief The RappCaffeWrapper ros node
class RappCaffeWrapper:

  ## @brief Default contructor
  #
  # Waits for services the node depends on and declares the callbacks of the node's services
  def __init__(self):    
	  
	  # Dependencies
    self.serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')
    if(not self.serv_topic):
      rospy.logerror("rapp_knowrob_wrapper_create_ontology_alias param not found")
    rospy.wait_for_service(self.serv_topic)
    
    self.serv_topic = rospy.get_param('rapp_knowrob_wrapper_register_image_object_to_ontology')
    if(not self.serv_topic):
      rospy.logerror("rapp_knowrob_wrapper_register_image_object_to_ontology param not found")
    rospy.wait_for_service(self.serv_topic)   

    #Declare Callbacks
    self.serv_topic = rospy.get_param("rapp_caffe_wrapper_image_classification")
    if(not self.serv_topic):
      rospy.logerror("rapp_caffe_wrapper_image_classification")
    self.serv=rospy.Service(self.serv_topic, imageClassificationSrv, self.imageClassificationDataHandler)  

    self.serv_topic = rospy.get_param("rapp_caffe_wrapper_get_ontology_class_equivalent")
    if(not self.serv_topic):
      rospy.logerror("rapp_caffe_wrapper_get_ontology_class_equivalent")
    self.serv=rospy.Service(self.serv_topic, ontologyClassBridgeSrv, self.ontologyClassBridgeDataHandler)     
    
    self.serv_topic = rospy.get_param("rapp_caffe_wrapper_register_image_to_ontology")
    if(not self.serv_topic):
      rospy.logerror("rapp_caffe_wrapper_register_image_to_ontology")
    self.serv=rospy.Service(self.serv_topic, registerImageToOntologySrv, self.registerImageToOntologyDataHandler)  

  ## @brief The imageClassificationSrv service callback
  # @param req [rapp_platform_ros_communications::imageClassificationSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::imageClassificationSrvResponse::Response&] The ROS service response    
  def imageClassificationDataHandler(self,req):
    res = imageClassificationSrvResponse()
    it = ImageClassification()
    res=it.classifyImage(req)
    return res

  ## @brief The ontologyClassBridgeSrv service callback
  # @param req [rapp_platform_ros_communications::ontologyClassBridgeSrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::ontologyClassBridgeSrvResponse::Response&] The ROS service response 
  def ontologyClassBridgeDataHandler(self,req):
    res = ontologyClassBridgeSrvResponse()
    it = OntologyClassBridge()
    res=it.getOntologyClassEquivalent(req)
    return res

  ## @brief The registerImageToOntologySrv service callback
  # @param req [rapp_platform_ros_communications::registerImageToOntologySrvRequest::Request&] The ROS service request
  # @return res [rapp_platform_ros_communications::registerImageToOntologySrvResponse::Response&] The ROS service response
  def registerImageToOntologyDataHandler(self,req):
    res = registerImageToOntologySrvResponse()
    it = ImageRegistrationToOntology()
    res=it.registerImage(req)
    return res

