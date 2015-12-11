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

from greek_support import *
from english_support import *
from greek_english_support import *
from sphinx4_wrapper import *
from rapp_tools import *
from sphinx4_configuration_params import *
from global_parameters import GlobalParams

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
  def __init__(self):

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
        return total_res

    conf_res = SpeechRecognitionSphinx4ConfigureSrvResponse()
    conf_res = self._configureSpeechRecognition(req)
    total_res.error = conf_res.error
    if conf_res.error != '':
        total_res.error = total_res.error + '\n' + conf_res.error
        return total_res

    spee_res = self._speechRecognition(req)
    total_res.words = spee_res.words
    total_res.error = spee_res.error
    return total_res

  ## Performs Sphinx4 speech recognition
  #
  # @param req  [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4SrvRequest] The speech recognition request
  # @return res [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4SrvResponse] The speech recognition response
  def _speechRecognition(self, req):
    res = SpeechRecognitionSphinx4SrvResponse()
    words = self._sphinx4.performSpeechRecognition(req.path, req.audio_source, req.user)
    rapp_print (words)
    # Error handling - Must be implemented with exceptions
    if len(words) == 1 and "Error:" in words[0]:
      res.error = words[0]
      res.words = []
      return res

    for word in words:
      if self._configuration_params._language != "en":
        rapp_print ("Word: #" + word + "#")
        if word == "" or word == '<unk>':
          continue
        res.words.append(self._word_mapping[word])
      else:
        res.words.append(word.replace("'"," "))

    return res;

  ## Performs Sphinx4 configuration
  #
  # @param req  [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4ConfigureSrvRequest] The sphinx configuration request
  # @return res [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4ConfigureSrvRequest] The sphinx configuration response
  def _configureSpeechRecognition(self, req):
    res = SpeechRecognitionSphinx4ConfigureSrvResponse()
    res.error = ''
    conf = {} # Dummy initialization
    reconfigure = True

    if self._configuration_params.equals(req):
      reconfigure = False
    self._configuration_params.makeEqualTo(req)

    if reconfigure == False:
      return res

    # English language
    if self._configuration_params._language == 'en':
      rapp_print ("Language set to English")
      # Whole dictionary utilization
      if len(self._configuration_params._words) == 0:
        rapp_print ("Generic model used")
        # success is either True (bool) or error (string)
        try:
            conf = self._english_support.getGenericConfiguration()
        except RappError as e:
            res.error = e.value
            return res
      # Limited dictionary utilization
      else:
        rapp_print ("Limited model used")
        # success is either True (bool) or error (string)
        try:
            conf = self._english_support.getLimitedVocebularyConfiguration(\
                self._configuration_params._words, \
                self._configuration_params._grammar, \
                self._configuration_params._sentences)
        except RappError as e:
            res.error = e.value
            return res

    # Greek language
    elif self._configuration_params._language == "el":
      rapp_print ("Language set to Greek")
      # Whole dictionary utilization
      if len(self._configuration_params._words) == 0:
        rapp_print ("Generic model used")
        # TODO
      # Limited dictionary utilization
      else:
        rapp_print ("Words to be recognized (" + \
            str(len(self._configuration_params._words)) + "):")
        # success is either True (bool) or error (string)
        try:
            [conf, eng_w] = self._greek_support.getLimitedVocebularyConfiguration(\
                self._configuration_params._words, \
                self._configuration_params._grammar, \
                self._configuration_params._sentences)
        except RappError as e:
            res.error = e.value
            return res

        self._word_mapping = {}
        for ew in eng_w:
          self._word_mapping[ew] = eng_w[ew]
        rapp_print (self._word_mapping)

    else:
      res.error = "Wrong language"
      return res

    # Actual sphinx4 configuration
    rapp_print ("Configuration: \n")
    rapp_print (conf)
    self._sphinx4.configureSphinx(conf)
    return res

# Main function
if __name__ == "__main__":
  rospy.init_node('SpeechRecognitionSphinx4')
  SpeechRecognitionSphinx4Node = SpeechRecognitionSphinx4()
  rospy.spin()
