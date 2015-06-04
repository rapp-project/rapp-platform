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

  def setup_two_words_voc(self):
    spreq = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spreq.language = 'gr'
    spreq.words = []
    spreq.words.append(u'όχι')
    spreq.words.append(u'ναι') 
    spreq.sentences = []
    spreq.sentences.append(u'όχι')
    spreq.sentences.append(u'ναι')
    spreq.grammar = []
    spreq.grammar.append(u'(όχι)')
    spreq.grammar.append(u'(ναι)')
    return spreq

  def __init__(self):   

    tberased = []
    
    self.conf_sp_ser_top = \
        rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
    self.denoising_service = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")

    self.conf_sp_ser = rospy.ServiceProxy(\
        self.conf_sp_ser_top,\
        SpeechRecognitionSphinx4TotalSrv)
    self.denoising = rospy.ServiceProxy(\
        self.denoising_service,\
        AudioProcessingDenoiseSrv)

    spreq = ""
    spreq = self.setup_two_words_voc()
    spreq.grammar = []
    spee_req = SpeechRecognitionSphinx4TotalSrvRequest()
   
    if not (len(sys.argv) == 2 or len(sys.argv) == 3):
      print "Invalid number of arguments"
      return

    if len(sys.argv) == 2:
      folder = sys.argv[1]
      files = [ f for f in listdir(folder) if isfile(join(folder, f)) ]
    else:
      folder = sys.argv[1]
      files = [sys.argv[2]]
    
    success = 0
    total = 0
    didnt_recognize = 0
    total_recognize = 0

    failed = []
    not_recognized = []
    response = []

    for f in files:
      
      toprint = ""

      response = []
      if not (("nai_" in f) or ("oxi_" in f)):
        continue
      #if "hard" in f:
        #continue
      if "nai_" in f:
        response.append( (u'ναι').encode('utf-8') )
      else:
        response.append( (u'όχι').encode('utf-8') )
           
      spee_req.language = spreq.language
      spee_req.words = spreq.words
      spee_req.grammar = spreq.grammar
      spee_req.user = 'etsardou'
      spee_req.sentences = spreq.sentences
      spee_req.path = folder + f
      spee_req.audio_source = 'headset' # The samples are already denoised

      # Perform power denoising
      #denoise_req = AudioProcessingDenoiseSrvRequest()
      #denoise_req.audio_file = spee_req.path
      #denoise_req.denoised_audio_file = spee_req.path + "denoised.wav"
      #spee_req.path = spee_req.path + "denoised.wav"
      #tberased.append(denoise_req.denoised_audio_file)
      #res = self.denoising(denoise_req)
      #########################
    
      res = self.conf_sp_ser(spee_req)

      toprint += os.path.basename(spee_req.path) + " : "
      for word in res.words:
        toprint += word + " "
      
      ok = False
      while not ok:
        ok = True
        for i in range(0, len(res.words) - 1):
          if res.words[i] == res.words[i + 1]:
            del res.words[i + 1]
            ok = False
            break;
        
      total += 1
      if len(res.words) == 1:
        total_recognize += 1
        if res.words[0] in response:
          success += 1
          toprint = bcolors.OKGREEN + toprint + bcolors.ENDC
        else:
          failed.append(f)
          toprint = bcolors.FAIL + toprint + bcolors.ENDC
      else:
        didnt_recognize += 1
        toprint = bcolors.WARNING + toprint + bcolors.ENDC
        not_recognized.append(f)
      
      print toprint,
      if total_recognize != 0:
        print " / " + str(success * 1.0 / total_recognize * 100.0) + "%",
      print " / " + str(didnt_recognize*1.0 / total * 100.0) + "%"
      
      # For debugging purposes
      #break
    
    for f in tberased:
      command = "rm " + f
      os.system(command)

    print "Not recognized:"
    for f in not_recognized:
      print f
    print "\nError:"
    for f in failed:
      print f
      

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()
  #rospy.spin()

