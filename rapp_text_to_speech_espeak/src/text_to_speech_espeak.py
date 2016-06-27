#!/usr/bin/env python2
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

    self.serv = rospy.Service(self.serv_topic, \
        TextToSpeechSrv, self.text_to_speech_callback)

  # The service callback
  def text_to_speech_callback(self, req):

    res = TextToSpeechSrvResponse()
    lang = req.language
    # Check for language
    if req.language == '':
        res.error = 'Language not specified'
        return res

    # Check for audio output
    if req.audio_output == '':
        res.error = 'Text2Speech: Audio output not specified'
        return res

    speed = 130 # upper limit = 180
    pitch = 50 # upper limit = 99
    if lang == 'el':
        lang = 'mb-gr2'
        speed = 160
        pitch = 99

    command = 'espeak -p ' + str(pitch) + ' -s ' + str(speed) + \
            ' -v ' + lang + ' --stdout \'' + req.text + '\' -w ' +\
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


