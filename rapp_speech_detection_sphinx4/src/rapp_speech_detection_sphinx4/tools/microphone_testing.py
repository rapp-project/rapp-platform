#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr

import rospy
import sys
from os import listdir
from os.path import isfile, join
import os

from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4Srv,
  SpeechRecognitionSphinx4SrvResponse,
  SpeechRecognitionSphinx4SrvRequest,
  SpeechRecognitionSphinx4ConfigureSrv,
  SpeechRecognitionSphinx4ConfigureSrvResponse,
  SpeechRecognitionSphinx4ConfigureSrvRequest,
  SpeechRecognitionSphinx4TotalSrv,
  SpeechRecognitionSphinx4TotalSrvResponse,
  SpeechRecognitionSphinx4TotalSrvRequest,
  
  AudioProcessingDenoiseSrv,
  AudioProcessingDenoiseSrvResponse,
  AudioProcessingDenoiseSrvRequest
  )

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class SpeechRecognitionTester: 

  def setup_words_voc(self):
    spreq = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spreq.language = 'en'
    spreq.words.append('i-am-here')
    spreq.words.append('you-are-there')
    spreq.sentences = spreq.words
    spreq.grammar = spreq.words
    return spreq

  def __init__(self):   

    self.conf_sp_ser_top = \
        rospy.get_param("rapp_speech_detection_sphinx4_total_topic")

    self.conf_sp_ser = rospy.ServiceProxy(\
        self.conf_sp_ser_top,\
        SpeechRecognitionSphinx4TotalSrv)

    self.spreq = ""
    self.spreq = self.setup_words_voc()
    self.spee_req = SpeechRecognitionSphinx4TotalSrvRequest()
   
    if not (len(sys.argv) == 2):
      print "Invalid number of arguments"
      return

    seconds = int(sys.argv[1])
    print "Seconds to record: " + str(seconds)
    cwd = os.getcwd() + "/"
    wav_file = cwd + "temp.wav"
    print "CWD = " + cwd
    cmd = "rec -r 16000 -e signed-integer -b 16 -c 1 temp.wav trim 0 " +\
            str(seconds)
    os.system(cmd)
    self.run(wav_file)
    #os.system("rm " + wav_file)
    
  def run(self, wav_file):
      
    toprint = ""
           
    self.spee_req.language = self.spreq.language
    self.spee_req.words = self.spreq.words
    self.spee_req.grammar = self.spreq.grammar
    self.spee_req.user = 'rapp'
    self.spee_req.sentences = self.spreq.sentences
    self.spee_req.path = wav_file
    self.spee_req.audio_source = 'headset' # The samples are already denoised

    res = self.conf_sp_ser(self.spee_req)
    print res
    for word in res.words:
      toprint += "'" + word + "' "
    print toprint

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()

