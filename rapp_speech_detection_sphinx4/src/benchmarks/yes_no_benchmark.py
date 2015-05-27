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

from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4Srv,
  SpeechRecognitionSphinx4SrvResponse,
  SpeechRecognitionSphinx4SrvRequest,
  SpeechRecognitionSphinx4ConfigureSrv,
  SpeechRecognitionSphinx4ConfigureSrvResponse,
  SpeechRecognitionSphinx4ConfigureSrvRequest,
  SpeechRecognitionSphinx4TotalSrv,
  SpeechRecognitionSphinx4TotalSrvResponse,
  SpeechRecognitionSphinx4TotalSrvRequest
  )

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
    self.conf_sp_ser = rospy.ServiceProxy(\
        '/ric/speech_detection_sphinx4_batch',\
        SpeechRecognitionSphinx4TotalSrv)
    
    spreq = ""
    spreq = self.setup_two_words_voc()
    spreq.grammar = []
    spee_req = SpeechRecognitionSphinx4TotalSrvRequest()
   
    if len(sys.argv) != 2:
      print "Invalid number of arguments"
      return

    folder = sys.argv[1]
    
    files = [ f for f in listdir(folder) if isfile(join(folder, f)) ]
    
    success = 0
    total = 0

    for f in files:
      response = ""
      if not (("nai_" in f) or ("oxi_" in f)):
        continue;
      if "nai_" in f:
        response = u'ναι'
      else:
        response = u'όχι'
           
      spee_req.language = spreq.language
      spee_req.words = spreq.words
      spee_req.grammar = spreq.grammar
      spee_req.user = 'etsardou'
      spee_req.sentences = spreq.sentences
      spee_req.path = folder + f
      spee_req.audio_source = 'headset' # The samples are already denoised
      res = self.conf_sp_ser(spee_req)
      print spee_req.path + " : ",
      for word in res.words:
        print word,
      
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
        if res.words[0] == response.encode('utf-8'):
          success += 1

      print str(success*1.0/total*100.0) + "%"
      

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()
  #rospy.spin()

