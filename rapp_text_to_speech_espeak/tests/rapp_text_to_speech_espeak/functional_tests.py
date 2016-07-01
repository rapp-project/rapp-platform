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

PKG='rapp_text_to_speech_espeak'

import sys
import unittest
import rospy
import rospkg
import os

from rapp_platform_ros_communications.srv import (
  TextToSpeechSrv,
  TextToSpeechSrvRequest
  )

class TextToSpeechFunc(unittest.TestCase):

    def test_english_normal(self):
        s_service = rospy.get_param("rapp_text_to_speech_espeak_topic")
        rospy.wait_for_service(s_service)
        stt_service = rospy.ServiceProxy(s_service, TextToSpeechSrv)
        req = TextToSpeechSrvRequest()
        req.audio_output =  '~/testing.wav'
        req.language = 'en'
        req.text = "This is a test"
        response = stt_service(req)
        self.assertEqual(response.error, "")
        self.assertEqual(os.path.isfile(os.path.expanduser("~/testing.wav")), True)
        os.system('rm ~/testing.wav')

    def test_english_no_lang(self):
        s_service = rospy.get_param("rapp_text_to_speech_espeak_topic")
        rospy.wait_for_service(s_service)
        stt_service = rospy.ServiceProxy(s_service, TextToSpeechSrv)
        req = TextToSpeechSrvRequest()
        req.audio_output =  '~/testing.wav'
        req.language = ''
        req.text = "This is a test"
        response = stt_service(req)
        self.assertNotEqual(response.error, "")
        self.assertNotEqual(os.path.isfile(os.path.expanduser("~/testing.wav")), True)
        os.system('rm ~/testing.wav')

    def test_greek(self):
        s_service = rospy.get_param("rapp_text_to_speech_espeak_topic")
        rospy.wait_for_service(s_service)
        stt_service = rospy.ServiceProxy(s_service, TextToSpeechSrv)
        req = TextToSpeechSrvRequest()
        req.audio_output =  '~/testing.wav'
        req.language = 'el'
        req.text = "This is a test"
        response = stt_service(req)
        self.assertEqual(response.error, "")
        self.assertEqual(os.path.isfile(os.path.expanduser("~/testing.wav")), True)
        os.system('rm ~/testing.wav')

    def test_error_language(self):
        s_service = rospy.get_param("rapp_text_to_speech_espeak_topic")
        rospy.wait_for_service(s_service)
        stt_service = rospy.ServiceProxy(s_service, TextToSpeechSrv)
        req = TextToSpeechSrvRequest()
        req.audio_output =  '~/testing.wav'
        req.language = 'elthisaa'
        req.text = "This is a test"
        response = stt_service(req)
        self.assertNotEqual(response.error, "")
        self.assertNotEqual(os.path.isfile(os.path.expanduser("~/testing.wav")), True)

    def test_error_text(self):
        s_service = rospy.get_param("rapp_text_to_speech_espeak_topic")
        rospy.wait_for_service(s_service)
        stt_service = rospy.ServiceProxy(s_service, TextToSpeechSrv)
        req = TextToSpeechSrvRequest()
        req.audio_output =  '~/testing.wav'
        req.language = 'en'
        req.text = ""
        response = stt_service(req)
        self.assertEqual(response.error, "")
        self.assertEqual(os.path.isfile(os.path.expanduser("~/testing.wav")), True)
        os.system('rm ~/testing.wav')

    def test_no_audio_output(self):
        s_service = rospy.get_param("rapp_text_to_speech_espeak_topic")
        rospy.wait_for_service(s_service)
        stt_service = rospy.ServiceProxy(s_service, TextToSpeechSrv)
        req = TextToSpeechSrvRequest()
        req.audio_output =  ''
        req.language = 'en'
        req.text = "Testing"
        response = stt_service(req)
        self.assertNotEqual(response.error, "")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'TextToSpeechFunc', TextToSpeechFunc)














