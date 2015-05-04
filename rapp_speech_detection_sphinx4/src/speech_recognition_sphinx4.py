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
import rospkg

from greek_support import *
from english_support import *
from sphinx4_wrapper import *

from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4Srv,
  SpeechRecognitionSphinx4SrvResponse,
  SpeechRecognitionSphinx4ConfigureSrv,
  SpeechRecognitionSphinx4ConfigureSrvResponse,
  )
    
from std_msgs.msg import ( 
  String 
  ) 

class SpeechRecognitionSphinx4: 
 
  # Constructor performing initializations
  def __init__(self):    

    self.sphinx4 = Sphinx4Wrapper()
    self.greek_support = GreekSupport()
    self.english_support = EnglishSupport()
    
    self.language = 'gr'
    self.words = []
    self.grammar = []
    self.sentences = []

    self.serv_topic = rospy.get_param("rapp_speech_detection_sphinx4_topic")
    self.serv_configuration_topic = \
        rospy.get_param("rapp_speech_detection_sphinx4_configuration_topic")
    
    if(not self.serv_topic):
      rospy.logerror("Sphinx4 Speech detection topic param not found")
    if(not self.serv_configuration_topic):
      rospy.logerror("Sphinx4 Speech detection configuration topic param not found")
   
    self.speech_recognition_service = rospy.Service(self.serv_topic, \
        SpeechRecognitionSphinx4Srv, self.speechRecognition)
    self.speech_recognition_configuration_service = rospy.Service( \
        self.serv_configuration_topic, SpeechRecognitionSphinx4ConfigureSrv, \
        self.configureSpeechRecognition)
   
    rospack = rospkg.RosPack()

    self.sphinx4_jars = rospack.get_path('rapp_sphinx4_java_libraries')   
    self.sphinx4_class_path = rospack.get_path('rapp_speech_detection_sphinx4')   
    
    total_path = ".:" + self.sphinx4_jars + "/sphinx4-core-1.0-SNAPSHOT.jar:" \
            + self.sphinx4_class_path + "/src"

    self.sphinx_configuration = { \
      'jar_path' : total_path, \
      'configuration_path' : self.sphinx4_jars+"/greekPack/default.config.xml", \
      'acoustic_model' : self.sphinx4_jars+"/acoustic_model", \
      'grammar_name' : 'hello', \
      'grammar_folder' : self.sphinx4_jars+"/greekPack/", \
      'dictionary' : self.sphinx4_jars + "/greekPack/custom.dict", \
      'language_model' : self.sphinx4_jars+"/greekPack/sentences.lm.dmp", \
      'grammar_disabled' : True
      }
    self.sphinx4.initializeSphinx(self.sphinx_configuration)
 
  # Service callback for handling speech recognition
  def speechRecognition(self, req):     
    res = SpeechRecognitionSphinx4SrvResponse()
    words = self.sphinx4.performSpeechRecognition(req.path.data)   
    for word in words:
      res.words.append(word)
   
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
        conf = self.english_support.getGenericConfiguration()
      # Limited dictionary utilization
      else:   
        print "Limited model used"
        conf = self.english_support.getLimitedVocebularyConfiguration(\
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
        conf = self.greek_support.getLimitedVocebularyConfiguration(\
            self.words, self.grammar, self.sentences)
   
    else:
      res.error = "Wrong language"
      return res
    
    # Actual sphinx4 configuration
    print "Configuration: \n"
    print conf
    self.sphinx4.configureSphinx(conf)
    return res

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionSphinx4')
  SpeechRecognitionSphinx4Node = SpeechRecognitionSphinx4()
  rospy.spin()
