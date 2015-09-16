#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

# rapp_speech_detection_sphinx4_detect_speech_topic: /rapp/rapp_speech_detection_sphinx4/speech_to_text
# rapp_speech_detection_sphinx4_configuration_topic: /rapp/rapp_speech_detection_sphinx4/configure
# rapp_speech_detection_sphinx4_total_topic: /rapp/rapp_speech_detection_sphinx4/batch_speech_to_text

class SpeechDetSphinx4Func(unittest.TestCase):

    def test_batchService(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
        req.words = ['ναι', 'όχι']
        req.grammar = []
        req.sentences = req.words
        req.path = aux + '/nao_wav_d05_a1.wav'
        req.audio_source = 'nao_wav_4_ch'
        req.user = 'rapp'

        response = test_service(req)
        self.assertEqual( response.error, '' )
        self.assertEqual( response.words, ['ναι', 'όχι', 'ναι'])

    def test_batchServiceErrorLanguage(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
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
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
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
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
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
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
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
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
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
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
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
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
        req.words = ['ναι', 'όχι']
        req.grammar = req.words
        req.sentences = req.words
        req.path = aux + '/nai_sample.wav'
        req.audio_source = 'nao_wav_1_ch_denoised'
        req.user = 'rapp'

        response = test_service(req)
        self.assertEqual( response.error, '' )
        self.assertEqual( response.words, ['ναι'] )

    def test_batchServiceOgg(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_auxiliary_files')

        conf_service = rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, SpeechRecognitionSphinx4TotalSrv)
        
        req = SpeechRecognitionSphinx4TotalSrvRequest()
        req.language = 'gr'
        req.words = ['ναι', 'όχι']
        req.grammar = []
        req.sentences = req.words
        req.path = aux + '/nao_ogg_d05_a1.ogg'
        req.audio_source = 'nao_ogg'
        req.user = 'rapp'

        response = test_service(req)
        self.assertEqual( response.error, '' )
        self.assertEqual( response.words, ['ναι'] )

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'SpeechDetSphinx4Func', SpeechDetSphinx4Func)

