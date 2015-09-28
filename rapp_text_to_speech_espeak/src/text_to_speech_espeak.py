#!/usr/bin/env python2
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

import rospy
import json
import sys
import os

from pylab import *
from scipy.io import wavfile

from rapp_platform_ros_communications.srv import (
  TextToSpeechSrv,
  TextToSpeechSrvResponse
  )

from rapp_exceptions import RappError

class TextToSpeechEspeak:

  def __init__(self):
    # Speech recognition service published
    self.serv_topic = rospy.get_param("rapp_text_to_speech_espeak_topic")
    if(not self.serv_topic):
        rospy.logerror("Text to speech espeak topic param not found")

    self.threads = rospy.get_param("rapp_text_to_speech_espeak_threads")
    if not self.threads:
        rospy.logerror("Text to speech espeak threads param not found")
        self.threads = 0

    self.serv = rospy.Service(self.serv_topic, \
        TextToSpeechSrv, self.text_to_speech_callback)
    for i in range(0, self.threads):
        rospy.Service(self.serv_topic + '_' + str(i), \
            TextToSpeechSrv, self.text_to_speech_callback)

  # The service callback  
  def text_to_speech_callback(self, req):
    
    res = TextToSpeechSrvResponse()
    lang = req.language
    # Check for language
    if req.language == '':
        lang = 'en'
    
    # Check for audio output
    if req.audio_output == '':
        res.error = 'Text2Speech: Audio output not specified'
        return res

    command = 'espeak -s 130 -v ' + lang + ' --stdout \'' + req.text + '\' -w ' +\
            req.audio_output
    output = os.system(command)
    if output != 0:
        res.error = "Error: Text to speech espeak malfunctioned. You have probably\
                given wrong language settings"
        return res

    # Transform the output to wav 16kHz mono
    os.system('sox ' + req.audio_output + ' -c 1 -r 16000 ' + req.audio_output + '2.wav')
    os.system('mv ' + req.audio_output + '2.wav ' + req.audio_output)
    return res

if __name__ == "__main__":
  rospy.init_node('text_to_speech_espeak_ros_node')
  text_to_speech_espeak_ros_node = TextToSpeechEspeak()
  rospy.spin()


