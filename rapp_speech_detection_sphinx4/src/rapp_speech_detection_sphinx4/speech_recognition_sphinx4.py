#!/usr/bin/env python
# -*- encode: utf-8 -*-

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr

import rospy
import sys
import subprocess
import time

from greek_support import *
from english_support import *
from sphinx4_wrapper import *

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

class SpeechRecognitionSphinx4(GlobalParams): 
 
  # Constructor performing initializations
  def __init__(self):    
    GlobalParams.__init__(self)

    self.sphinx4 = Sphinx4Wrapper()
    self.greek_support = GreekSupport()
    self.english_support = EnglishSupport()
    self.word_mapping = {}
    
    self.language = 'gr'
    self.words = []
    self.grammar = []
    self.sentences = []

    self.serv_topic = rospy.get_param("rapp_speech_detection_sphinx4_detect_speech_topic")
    self.serv_configuration_topic = \
        rospy.get_param("rapp_speech_detection_sphinx4_configuration_topic")
    self.serv_batch_topic = \
        rospy.get_param("rapp_speech_detection_sphinx4_total_topic")

    self.use_db_authentication = rospy.get_param(\
        "rapp_speech_detection_sphinx4_use_db_authentication")
   
    #---------------------------Check db authentication------------------------#
    if self.use_db_authentication == True:
      self.serv_db_topic = rospy.get_param("rapp_mysql_wrapper_user_fetch_data_topic")
      self.authentication_service = rospy.ServiceProxy(\
              self.serv_db_topic, fetchDataSrv)

    if(not self.serv_topic):
      rospy.logerror("Sphinx4 Speech detection topic param not found")
    if(not self.serv_configuration_topic):
      rospy.logerror("Sphinx4 Speech detection configuration topic param not found")
    if(not self.serv_batch_topic):
      rospy.logerror("Sphinx4 Speech detection batch topic param not found")
    if(not self.use_db_authentication):
      rospy.logerror("Sphinx4 Seech Detection use authentication param not found")

    self.speech_recognition_service = rospy.Service(self.serv_topic, \
        SpeechRecognitionSphinx4Srv, self.speechRecognition)
    self.speech_recognition_configuration_service = rospy.Service( \
        self.serv_configuration_topic, SpeechRecognitionSphinx4ConfigureSrv, \
        self.configureSpeechRecognition)
    self.speech_recognition_batch_service = rospy.Service( \
        self.serv_batch_topic, SpeechRecognitionSphinx4TotalSrv, \
        self.speechRecognitionBatch)
    
    total_path = ".:" + self.sphinx_jar_files_url + "/sphinx4-core-1.0-SNAPSHOT.jar:" \
            + self.sphinx_package_url + "/src"

    self.sphinx_configuration = { \
      'jar_path' : total_path, \
      'configuration_path' : self.language_models_url + "/greekPack/default.config.xml", \
      'acoustic_model' : self.acoustic_models_url , \
      'grammar_name' : 'hello', \
      'grammar_folder' : self.language_models_url + "/greekPack/", \
      'dictionary' : self.language_models_url + "/greekPack/custom.dict", \
      'language_model' : self.language_models_url + "/greekPack/sentences.lm.dmp", \
      'grammar_disabled' : True
      }
    self.sphinx4.initializeSphinx(self.sphinx_configuration)
 
  # Service callback for handling sphinx4 configuration AND speech recognition
  def speechRecognitionBatch(self, req):

    total_res = SpeechRecognitionSphinx4TotalSrvResponse()

    #-------------------------Check with database-------------------------#
    if self.use_db_authentication == True:
      req_db = fetchDataSrv()
      req_db.req_cols=["username"]
      entry1=["username", req.user]
      req_db.where_data=[StringArrayMsg(s=entry1)]

      resp = self.authentication_service(req_db.req_cols, req_db.where_data)
      if resp.success.data != True or len(resp.res_data) == 0: 
        total_res.error = "Non authenticated user"
        return total_res
          
    conf_req = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spee_req = SpeechRecognitionSphinx4SrvRequest()

    conf_req.language = req.language
    conf_req.words = req.words
    conf_req.grammar = req.grammar
    conf_req.sentences = req.sentences

    conf_res = SpeechRecognitionSphinx4ConfigureSrvResponse()
    conf_res = self.configureSpeechRecognition(conf_req)
    total_res.error = conf_res.error
    if conf_res.error != '':
        total_res.error = total_res.error + '\n' + conf_res.error
        return total_res

    spee_req.path = req.path
    spee_req.audio_source = req.audio_source
    spee_req.user = req.user
    spee_res = self.speechRecognition(spee_req)
    total_res.words = spee_res.words
    total_res.error = spee_res.error
    return total_res

  # Service callback for handling speech recognition
  def speechRecognition(self, req):     
    res = SpeechRecognitionSphinx4SrvResponse()
    words = self.sphinx4.performSpeechRecognition(req.path, req.audio_source, req.user) 
    print words
    # Error handling - Must be implemented with exceptions
    if len(words) == 1 and "Error:" in words[0]:
      res.error = words[0]
      res.words = []
      return res
    
    for word in words:
      if self.language != "en":
        print "Word: #" + word + "#"
        if word == "" or word == '<unk>':
          continue
        res.words.append(self.word_mapping[word])
      else:
        res.words.append(word.replace("'"," "))
    
    return res;  

  # Service callback dedicated for Sphinx4 configuration
  def configureSpeechRecognition(self, req):
    res = SpeechRecognitionSphinx4ConfigureSrvResponse()
    
    conf = {} # Dummy initialization
    reconfigure = False
    
    if self.language != req.language:
      reconfigure = True
    if self.words != req.words:
      reconfigure = True
    if self.grammar != req.grammar:
      reconfigure = True
    if self.sentences != req.sentences:
      reconfigure = True
    self.language = req.language
    self.words = req.words
    self.grammar = req.grammar
    self.sentences = req.sentences

    if reconfigure == False:
      return res

    # English language
    if self.language == 'en':
      print "Language set to English"
      # Whole dictionary utilization
      if len(self.words) == 0: 
        print "Generic model used"
        [conf, success] = self.english_support.getGenericConfiguration()
      # Limited dictionary utilization
      else:   
        print "Limited model used"
        [conf, success] = self.english_support.getLimitedVocebularyConfiguration(\
            self.words, self.grammar, self.sentences)

    # Greek language
    elif self.language == "gr":
      print "Language set to Greek"
      # Whole dictionary utilization
      if len(self.words) == 0:
        print "Generic model used"
        # TODO
      # Limited dictionary utilization
      else:
        print "Words to be recognized (" + str(len(self.words)) + "):"
        [conf, eng_w, success] = self.greek_support.getLimitedVocebularyConfiguration(\
            self.words, self.grammar, self.sentences)
        for ew in eng_w:
          self.word_mapping[ew] = eng_w[ew]
        print self.word_mapping
   
    else:
      res.error = "Wrong language"
      return res
    
    # sanity check
    if success != True:
        res.error = success
        return res

    # Actual sphinx4 configuration
    print "Configuration: \n"
    print conf
    self.sphinx4.configureSphinx(conf)
    if success == True:
        res.error = ''
    else:
        res.error = success
    return res

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionSphinx4')
  SpeechRecognitionSphinx4Node = SpeechRecognitionSphinx4()
  rospy.spin()
