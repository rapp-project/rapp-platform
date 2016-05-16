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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr
 
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import shutil
from os.path import expanduser
from app_error_exception import AppError
from rapp_platform_ros_communications.srv import (
  createOntologyAliasSrv,
  createOntologyAliasSrvRequest,
  createOntologyAliasSrvResponse,
  registerImageToOntologySrv,
  registerImageToOntologySrvResponse,
  registerImageToOntologySrvRequest,
  registerImageObjectToOntologySrv,
  registerImageObjectToOntologySrvRequest,
  registerImageObjectToOntologySrvResponse
  )


## @class ImageRegistrationToOntology
# @brief Contains the necessary functions for registering images to the ontology
class ImageRegistrationToOntology:
  
  ## @brief Implements the getCloudAgentServiceTypeAndHostPort service main function
  # @param req [rapp_platform_ros_communications::registerImageToOntologySrvRequest::Request&] The ROS service request
  #
  # @return res [rapp_platform_ros_communications::registerImageToOntologySrvResponse::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception IOError
  # @exception Exception AppError
  # @exception Exception KeyError
  def registerImage(self,req):
    try:      
      res = registerImageToOntologySrvResponse()  
      ontologyAlias=self.getUserOntologyAlias(req.username)         
      baseDestinationFolder = rospy.get_param("user_images_folder")+req.username+"_"+ontologyAlias+"/"
      ontologyName=self.registerImageToOntology(req,baseDestinationFolder)
      ontologyName=ontologyName.split('#')[1]  
      
      baseDestinationFolderUserAdjusted=expanduser("~")+baseDestinationFolder
      print baseDestinationFolderUserAdjusted
      if not os.path.exists(baseDestinationFolderUserAdjusted):
        os.makedirs(baseDestinationFolderUserAdjusted)
         
      shutil.copy(req.imagePath, baseDestinationFolderUserAdjusted+ontologyName)
      res.object_entry=ontologyName
      res.success=True  
    
    except IndexError, e:
      res.trace.append("IndexError: " +str(e))
      res.error="IndexError: "+str(e)
      res.success=False
    except IOError, e:
      res.success=False
      res.trace.append("IOError: "+str(e))
      res.error="IOError: "+str(e)
    except AppError as e:
      AppError.passErrorToRosSrv(e,res)     
    except KeyError, e:
      res.success=False
      res.trace.append('"KeyError, probably caffe class does not exist or no ontology equivalent exists for "%s"' % str(e))
      res.error='"KeyError, probably caffe class does not exist or no ontology equivalent exists for "%s"' % str(e)
    return res
    

  ## @brief Gets the users ontology alias and if it doesnt exist it creates it  
  # @param username [string] The user's username
  #
  # @return ontologyAlias [string] The user's ontology alias
  # @exception Exception AppError
  def getUserOntologyAlias(self,username):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')      
    knowrob_service = rospy.ServiceProxy(serv_topic, createOntologyAliasSrv)
    createOntologyAliasReq = createOntologyAliasSrvRequest()
    createOntologyAliasReq.username=username
    createOntologyAliasResponse = knowrob_service(createOntologyAliasReq)
    if(createOntologyAliasResponse.success!=True):
      raise AppError(createOntologyAliasResponse.error, createOntologyAliasResponse.trace)      
    return createOntologyAliasResponse.ontology_alias

  ## @brief Calls the knowrob_wrapper service that registers the image to the ontology  
  # @param req [rapp_platform_ros_communications::registerImageToOntologySrvRequest::Request&] The ROS service request
  # @param baseDestinationFolder [string] The path to the image
  #
  # @return registerImageObjectToOntologyResponse.object_entry [string] The ontology alias image entry
  # @exception Exception AppError
  def registerImageToOntology(self,req,baseDestinationFolder):
    serv_topic = rospy.get_param('rapp_knowrob_wrapper_register_image_object_to_ontology')
    knowrob_service = rospy.ServiceProxy(serv_topic, registerImageObjectToOntologySrv)
    registerImageObjectToOntologyReq = registerImageObjectToOntologySrvRequest()
    registerImageObjectToOntologyReq.user_ontology_alias=self.getUserOntologyAlias(req.username)    
    registerImageObjectToOntologyReq.image_path=baseDestinationFolder
    registerImageObjectToOntologyReq.caffe_class=req.caffeClass
    registerImageObjectToOntologyReq.timestamp=int(time.time())
    registerImageObjectToOntologyReq.object_ontology_class=req.ontologyClass
    registerImageObjectToOntologyResponse = knowrob_service(registerImageObjectToOntologyReq)
    if(registerImageObjectToOntologyResponse.success!=True):     
      registerImageObjectToOntologyResponse.trace.append("Error in registering the image to the ontology")
      raise AppError(registerImageObjectToOntologyResponse.error+"Error in registering the image to the ontology",registerImageObjectToOntologyResponse.trace)   
    return registerImageObjectToOntologyResponse.object_entry

