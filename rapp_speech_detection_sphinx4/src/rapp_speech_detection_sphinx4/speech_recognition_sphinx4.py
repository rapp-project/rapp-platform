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

# Authors: Athanassios Kintsakis, Aris Thallas, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, aris.thallas@{iti.gr, gmail.com}, etsardou@iti.gr

import rospy
import sys
import subprocess
import time

from rapp_utilities import RappUtilities

from greek_support import *
from english_support import *
from greek_english_support import *
from sphinx4_wrapper import *
from sphinx4_configuration_params import *
from global_parameters import GlobalParams

from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4Srv,
  SpeechRecognitionSphinx4SrvResponse,
  SpeechRecognitionSphinx4SrvRequest,
  SpeechRecognitionSphinx4ConfigureSrv,
  SpeechRecognitionSphinx4ConfigureSrvResponse,
  SpeechRecognitionSphinx4ConfigureSrvRequest,
  SpeechRecognitionSphinx4TotalSrv,
  SpeechRecognitionSphinx4TotalSrvResponse
  )

from rapp_platform_ros_communications.srv import (
  fetchDataSrv,
  fetchDataSrvRequest
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from std_msgs.msg import (
  String
  )

## @class SpeechRecognitionSphinx4
# @brief Provides a complete Rapp Sphinx Entity.
#
# Maintains a complete Rapp Sphinx Entity for the
# speech_recognition_sphinx4_handler_node.SpeechRecognitionSphinx4HandlerNode
# to perform the speech recognition.
class SpeechRecognitionSphinx4():

  ## Constructor performing initializations
  def __init__(self, configurationName=None):

    ## Contains global Sphinx parameters
    #
    # (see global_parameters.GlobalParams)
    self._globalParams = GlobalParams()

    ## The sphinx wrapper communicates with the actual Sphinx.java process
    #
    # (see sphinx4_wrapper.Sphinx4Wrapper)
    self._sphinx4 = Sphinx4Wrapper()
    ## Greek_support creates necessary files for Greek speech recognition
    #
    # (see greek_support.GreekSupport)
    self._greek_support = GreekEnglishSupport()
    ## English creates necessary files for english speech recognition
    #
    # (see english_support.EnglishSupport)
    self._english_support = EnglishSupport()
    ## The Sphinx configuration parameters
    #
    # (see sphinx4_configuration_params.SphinxConfigurationParams)
    self._configuration_params = SphinxConfigurationParams()

    ## A dictionary to transform the englified greek words to actual greek words
    self._word_mapping = {}

    if configurationName != None:
      self._createPreconfiguration( configurationName )

    ## Defines whether database authentication should be used
    self._use_db_authentication = rospy.get_param(\
        "rapp_speech_detection_sphinx4_use_db_authentication")

    #---------------------------Check db authentication------------------------#
    if self._use_db_authentication == True:
      serv_db_topic = \
          rospy.get_param("rapp_mysql_wrapper_user_fetch_data_topic")
      ## The database authentication service client
      #
      # Used to identify user performed the speech recognition request.
      # (see mysql_wrapper.MySQLdbWrapper.fetchData)
      self._authentication_service = rospy.ServiceProxy(\
              serv_db_topic, fetchDataSrv)

    if(not self._use_db_authentication):
      rospy.logerror("Sphinx4 Seech Detection use authentication param not found")


  ## @brief Requests the configuration's sha1 hash
  #
  # Hash is used to identify common request configurations for proper
  # subprocess selection.
  # (Requests with common requests do not require reconfiguration reducing
  # computation time)
  #
  # @return hexdigest [string] The hash digest containing only hexadecimal digits
  def getConfigurationHash(self):
    return self._configuration_params.getHash()

  ## @brief Create the requested preconfiguration
  #
  # Creates the configuration via the name requested from
  # rapp_speech_detection_sphinx4::cfg::sphinx4_wrapper_params.yaml
  #
  # @param configurationName [string] The preconfiguration name
  def _createPreconfiguration(self, configurationName):
    RappUtilities.rapp_print( "Creating preconfiguration: " + configurationName )
    tempConf = SphinxConfigurationParams( configurationName )

    req = SpeechRecognitionSphinx4ConfigureSrvRequest()
    req.language = tempConf._language
    req.words = tempConf._words
    req.grammar = tempConf._grammar
    req.sentences = tempConf._sentences

    self._configureSpeechRecognition( req )

  ## Performs Sphinx4 configuration and speech recognition
  #
  # @param req  [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4TotalSrvRequest] The speech recognition request
  # @return res [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4TotalSrvResponse] The speech recognition response
  def speechRecognitionBatch(self, req):

    total_res = SpeechRecognitionSphinx4TotalSrvResponse()

    #-------------------------Check with database-------------------------#
    if self._use_db_authentication == True:
      req_db = fetchDataSrv()
      req_db.req_cols=["username"]
      entry1=["username", req.user]
      req_db.where_data=[StringArrayMsg(s=entry1)]

      resp = self._authentication_service(req_db.req_cols, req_db.where_data)
      if resp.success.data != True or len(resp.res_data) == 0:
        total_res.error = "Non authenticated user"
        RappUtilities.rapp_print(total_res.error, 'ERROR')
        return total_res

    RappUtilities.rapp_print('Configuring Sphinx')
    conf_res = SpeechRecognitionSphinx4ConfigureSrvResponse()
    conf_res = self._configureSpeechRecognition(req)
    total_res.error = conf_res.error
    if conf_res.error != '':
        total_res.error = total_res.error + '\n' + conf_res.error
        RappUtilities.rapp_print(total_res.error, 'ERROR')
        return total_res

    RappUtilities.rapp_print('Performing recognition')
    spee_res = self._speechRecognition(req)
    total_res.words = spee_res.words
    total_res.error = spee_res.error
    RappUtilities.rapp_print(total_res.words)
    return total_res

  ## Performs Sphinx4 speech recognition
  #
  # @param req  [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4SrvRequest] The speech recognition request
  # @return res [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4SrvResponse] The speech recognition response
  def _speechRecognition(self, req):
    res = SpeechRecognitionSphinx4SrvResponse()
    words = self._sphinx4.performSpeechRecognition(req.path, req.audio_source, req.user)
    RappUtilities.rapp_print (words)
    # Error handling - Must be implemented with exceptions
    if len(words) == 1 and "Error:" in words[0]:
      res.error = words[0]
      res.words = []
      return res

    for word in words:
      RappUtilities.rapp_print ("Word: #" + word + "#")
      if word == "" or word == '<unk>':
        continue
      res.words.append(self._word_mapping[word].replace("'", " "))

    return res;

  ## Choose the language support based on the request language
  #
  # @param req  [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4ConfigureSrvRequest] The sphinx configuration request
  #
  # @return support [language_support::LanguageSupport] A child class of the LanguageSupport depending on the requested language
  def _selectLanguageSupport(self,  req):
    if self._configuration_params._language == 'en':
      RappUtilities.rapp_print ("Language set to English")
      return self._english_support
    elif self._configuration_params._language == 'el':
      RappUtilities.rapp_print ("Language set to Greek")
      return self._greek_support
    else:
      raise RappError("Wrong Language")

  ## Performs Sphinx4 configuration
  #
  # @param req  [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4ConfigureSrvRequest] The sphinx configuration request
  # @return res [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4ConfigureSrvRequest] The sphinx configuration response
  def _configureSpeechRecognition(self, req):
    res = SpeechRecognitionSphinx4ConfigureSrvResponse()
    res.error = ''
    conf = {} # Dummy initialization
    reconfigure = True

    if self._configuration_params.equalsRequest(req):
      reconfigure = False
    self._configuration_params.makeEqualToRequest(req)

    if reconfigure == False:
      RappUtilities.rapp_print('Skipping Configuration')
      return res
    RappUtilities.rapp_print('Recognition Configuration')

    try:
      support = self._selectLanguageSupport(req)
    except RappError as e:
      res.error = e.value
      return res

    try:
      conf = self._createSupportConfiguration(support, res)
    except RappError as e:
      res.error = e.value
      return res

    # Actual sphinx4 configuration
    RappUtilities.rapp_print ("Configuration: \n")
    RappUtilities.rapp_print (conf)
    self._sphinx4.configureSphinx(conf)
    return res


  ## Get Sphinx configuration paths from Language Support
  #
  # @param support [language_support::LanguageSupport] A child class of the LanguageSupport depending on the requested language
  # @param req [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4ConfigureSrvRequest] The sphinx configuration request
  #
  # @return conf [dictionary] The Sphinx configuration files' paths
  def _createSupportConfiguration(self, support, res):
    # Whole dictionary utilization
    if len(self._configuration_params._words) == 0:
      RappUtilities.rapp_print ("Generic model used")
      # success is either True (bool) or error (string)
      try:
        conf = support.getGenericConfiguration()
      except RappError as e:
        raise RappError( e.value )
    # Limited dictionary utilization
    else:
      RappUtilities.rapp_print ("Limited model used")
      # success is either True (bool) or error (string)
      try:
        [conf, mapping] = support.getLimitedVocebularyConfiguration(\
          self._configuration_params._words, \
          self._configuration_params._grammar, \
          self._configuration_params._sentences)
      except RappError as e:
        raise RappError( e.value )

      self._word_mapping = {}
      for ew in mapping:
        self._word_mapping[ew] = mapping[ew]
      RappUtilities.rapp_print (self._word_mapping)

    return conf


# Main function
if __name__ == "__main__":
  rospy.init_node('SpeechRecognitionSphinx4')
  SpeechRecognitionSphinx4Node = SpeechRecognitionSphinx4()
  rospy.spin()
