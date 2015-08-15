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
    spreq.words = []
    spreq.words.append(u'αυθυποστασία')
    spreq.words.append(u'εκκένωση') 
    spreq.words.append(u'επιστήμονας') 
    spreq.words.append(u'εξαιρετικά') 
    spreq.words.append(u'εκστρατεία') 
    spreq.words.append(u'λαύρα') 
    spreq.words.append(u'ψευδός') 
    spreq.words.append(u'πούρο') 
    spreq.words.append(u'αίσχος') 
    spreq.words.append(u'αυγό') 
    spreq.words.append(u'ψεύτης') 
    spreq.words.append(u'λοιμός') 
    spreq.words.append(u'κλοπιμαίο') 
    spreq.words.append(u'αγγούρι') 
    spreq.words.append(u'γκαρίζεις') 
    spreq.words.append(u'κάλτσα') 
    spreq.words.append(u'τζαμί') 
    spreq.words.append(u'αίμα') 
    spreq.words.append(u'χάσμα') 
    spreq.words.append(u'είλωτας') 
    spreq.words.append(u'κλοπιμαίο') 
    spreq.words.append(u'ξαφρίζω') 
    spreq.words.append(u'ποινή') 
    spreq.words.append(u'ωριαίος') 
    spreq.words.append(u'Καλυτερεύω') 
    spreq.words.append(u'διαίρεση') 
    spreq.words.append(u'καΐδι') 
    spreq.words.append(u'εύστοχος') 
    spreq.words.append(u'διαίρεση') 
    spreq.sentences = spreq.words
    spreq.grammar = spreq.words
    return spreq

  def __init__(self):   

    self.conf_topic = \
        rospy.get_param("rapp_speech_detection_sphinx4_configuration_topic")

    self.conf_service = rospy.ServiceProxy(\
        self.conf_topic,\
        SpeechRecognitionSphinx4ConfigureSrv)

    self.spreq = ""
    self.spreq = self.setup_words_voc()
    self.spreq.grammar = []
     
    res = self.conf_service(self.spreq)
      

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()

