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

from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4Srv,
  SpeechRecognitionSphinx4SrvResponse,
  SpeechRecognitionSphinx4SrvRequest,
  SpeechRecognitionSphinx4ConfigureSrv,
  SpeechRecognitionSphinx4ConfigureSrvResponse,
  SpeechRecognitionSphinx4ConfigureSrvRequest
  )

class SpeechRecognitionTester: 
 
  def __init__(self):    
    spreq = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spreq.language = 'gr'
    spreq.words = []
    spreq.words.append(u'όχι')
    spreq.words.append(u'ναι')
    spreq.words.append(u'πάνω')
    spreq.words.append(u'χάνω')
    spreq.words.append(u'κάτω')
    spreq.words.append(u'επανέλαβε')
    spreq.words.append(u'αριστερά')
    spreq.words.append(u'προχώρα')
    spreq.words.append(u'σου')
    spreq.words.append(u'γειά')
    spreq.words.append(u'πως')
    spreq.words.append(u'σε')
    spreq.words.append(u'λένε')
    spreq.words.append(u'ρε')
    spreq.words.append(u'λέμε')

    spreq.sentences = []
    spreq.sentences.append(u'όχι')
    spreq.sentences.append(u'ναι')
    spreq.sentences.append(u'πάνω')
    spreq.sentences.append(u'χάνω')
    spreq.sentences.append(u'κάτω')
    spreq.sentences.append(u'επανέλαβε')
    spreq.sentences.append(u'αριστερά')
    spreq.sentences.append(u'προχώρα')
    spreq.sentences.append(u'σου')
    spreq.sentences.append(u'γειά')
    spreq.sentences.append(u'πως')
    spreq.sentences.append(u'σε')
    spreq.sentences.append(u'λένε')
    spreq.sentences.append(u'ρε')
    spreq.sentences.append(u'λέμε')

    spreq.grammar = []
    spreq.grammar.append(u'(όχι)')
    spreq.grammar.append(u'(ναι)')
    spreq.grammar.append(u'(πάνω)')
    spreq.grammar.append(u'(χάνω)')
    spreq.grammar.append(u'(κάτω)')
    spreq.grammar.append(u'(επανέλαβε)')
    spreq.grammar.append(u'(αριστερά)')
    spreq.grammar.append(u'(προχώρα)')
    spreq.grammar.append(u'(σου)')
    spreq.grammar.append(u'(γειά)')
    spreq.grammar.append(u'(πως)')
    spreq.grammar.append(u'(σε)')
    spreq.grammar.append(u'(λένε)')
    spreq.grammar.append(u'(ρε)')
    spreq.grammar.append(u'(λέμε)')

    configure_service = rospy.ServiceProxy(\
        '/ric/speech_detection_sphinx4_configure',\
        SpeechRecognitionSphinx4ConfigureSrv)
    res = configure_service(spreq)
    print "Configuration done!\n"

    reqspeak = SpeechRecognitionSphinx4SrvRequest()
    conf_sp_ser = rospy.ServiceProxy(\
        '/ric/speech_detection_sphinx4',\
        SpeechRecognitionSphinx4Srv)
    reqspeak.path.data = "/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform-supplementary-material/rapp_sphinx4_java_libraries/recordings/nao_all_words.ogg"
    res = conf_sp_ser(reqspeak)
    print res

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()
  #rospy.spin()

