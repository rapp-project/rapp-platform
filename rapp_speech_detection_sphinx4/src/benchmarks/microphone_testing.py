#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
    spreq.language = 'gr'
    spreq.words = [u'όχι', u'ναι']
    spreq.sentences = spreq.words
    spreq.grammar = []
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
    cmd = "arecord -r 16000 -d " + str(seconds) + " temp.wav"
    os.system(cmd)
    self.run(wav_file)
    #os.system("rm " + wav_file)
    
  def run(self, wav_file):
      
    toprint = ""
           
    self.spee_req.language = self.spreq.language
    self.spee_req.words = self.spreq.words
    self.spee_req.grammar = self.spreq.grammar
    self.spee_req.user = 'etsardou'
    self.spee_req.sentences = self.spreq.sentences
    self.spee_req.path = wav_file
    self.spee_req.audio_source = 'headset' # The samples are already denoised
    res = self.conf_sp_ser(self.spee_req)
    for word in res.words:
      toprint +="'" + word + "' "
    print toprint

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()

