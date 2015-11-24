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

PKG='rapp_speech_detection_sphinx4'

import sys
import unittest
import rospy
import roslib
import rospkg

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

class SpeechDetSphinx4Func(unittest.TestCase):

    def test_batchService(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.grammar = []
        req.sentences = req.words
        req.path = aux + '/nao_wav_d05_a1.wav'
        req.audio_source = 'nao_wav_4_ch'
        req.user = 'rapp'

        response = test_service(req)
        self.assertEqual( response.error, '' )
        if ( response.words != ['ναι', 'όχι', 'ναι'] ):
            self.assertEqual( 'Word miss-match', '' )

    def test_batchService_stress(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        for i in range(0, 10):
          req = SpeechRecognitionSphinx4TotalSrvRequest()
          req.language = 'el'
          req.words = ['ναι', 'όχι']
          req.grammar = []
          req.sentences = req.words
          req.path = aux + '/nao_wav_d05_a1.wav'
          req.audio_source = 'nao_wav_4_ch'
          req.user = 'rapp'

          response = test_service(req)
          self.assertEqual( response.error, '' )
          if ( response.words != ['ναι', 'όχι', 'ναι'] ):
              self.assertEqual( 'Word miss-match', '' )

    def test_batchServiceErrorLanguage(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'en'
        req.words = ['ναι', 'όχι']
        req.grammar = []
        req.sentences = req.words
        req.path = aux + '/nao_wav_d05_a1.wav'
        req.audio_source = 'nao_wav_4_ch'
        req.user = 'rapp'

        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_batchServiceErrorSentences(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.grammar = []
        req.sentences = req.words + ['mine']
        req.path = aux + '/nao_wav_d05_a1.wav'
        req.audio_source = 'nao_wav_4_ch'
        req.user = 'rapp'

        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_batchServiceErrorGrammar(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.grammar = req.words + ['mine']
        req.sentences = req.words
        req.path = aux + '/nao_wav_d05_a1.wav'
        req.audio_source = 'nao_wav_4_ch'
        req.user = 'rapp'

        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_batchServiceWrongFile(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.grammar = req.words
        req.sentences = req.words
        req.path = aux + '/nao_wav_d05_a1_not.wav'
        req.audio_source = 'nao_wav_4_ch'
        req.user = 'rapp'

        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_batchServiceWrongUser(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.grammar = req.words
        req.sentences = req.words
        req.path = aux + '/nao_wav_d05_a1.wav'
        req.audio_source = 'nao_wav_4_ch'
        req.user = 'not_existing_user'

        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_batchServiceWrongType(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.grammar = req.words
        req.sentences = req.words
        req.path = aux + '/nao_wav_d05_a1.wav'
        req.audio_source = 'nao_wav_7_ch'
        req.user = 'rapp'

        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_batchService1Ch(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.sentences = req.words
        req.path = aux + '/nai_sample.wav'
        req.audio_source = 'nao_wav_1_ch_denoised'
        req.user = 'rapp'

        response = test_service(req)
        self.assertEqual( response.error, '' )
        if ( response.words != ['ναι'] ):
            self.assertEqual( 'Word miss-match', '' )

    def test_batchServiceOgg(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['ναι', 'όχι']
        req.grammar = []
        req.sentences = req.words
        req.path = aux + '/nao_ogg_d05_a1.ogg'
        req.audio_source = 'nao_ogg'
        req.user = 'rapp'

        response = test_service(req)
        self.assertEqual( response.error, '' )
        if ( response.words != ['ναι', 'όχι', 'ναι'] ):
            self.assertEqual( 'Word miss-match', '' )

    def test_batchServiceMultilanguage(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
            "rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
            conf_service, SpeechRecognitionSphinx4TotalSrv)

        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'el'
        req.words = ['στειλε', 'mail']
        req.grammar = []
        req.sentences = req.words
        req.path = aux + '/steile_mail.wav'
        req.audio_source = 'headset'
        req.user = 'rapp'

        response = test_service(req)
        self.assertEqual( response.error, '' )
        if ( response.words != ['στειλε', 'mail'] ):
            self.assertEqual( 'Word miss-match', '' )

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'SpeechDetSphinx4Func', SpeechDetSphinx4Func)

