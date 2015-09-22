#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG='rapp_speech_detection_google'

import sys
import unittest
import rospy
import rospkg

from rapp_platform_ros_communications.srv import (
  SpeechToTextSrv,
  SpeechToTextSrvRequest
  )

class SpeechToTextFunc(unittest.TestCase):

    def test_wavFile(self):
        rospack = rospkg.RosPack()
        google_service = rospy.get_param(\
                "rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_testing_tools') + \
                '/testing_tools/test_data/yes-no.wav'
        req.audio_type = 'nao_wav_1_ch'
        req.user = 'rapp'
        req.language = 'en'
        response = stt_service(req)
        words_basic = len(response.words)

        # Check number of words 
        self.assertEqual( words_basic, 2)
        self.assertEqual( 'yes' in response.words, True)
        self.assertEqual( 'no' in response.words, True)

    def test_wavFile_2(self):
        rospack = rospkg.RosPack()
        google_service = rospy.get_param(\
                "rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_testing_tools') + \
                '/testing_tools/test_data/speech_detection_samples/recording_monday.ogg'
        req.audio_type = 'nao_ogg'
        req.user = 'rapp'
        req.language = 'en'
        response = stt_service(req)
        words_basic = len(response.words)

        # Check number of words 
        self.assertEqual( words_basic, 1)
        self.assertEqual( 'Monday' in response.words, True)

    def test_imageFile(self):
        rospack = rospkg.RosPack()
        google_service = rospy.get_param(\
                "rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_testing_tools') + \
                '/testing_tools/test_data/Lenna.png'
        req.audio_type = 'nao_wav_1_ch'
        req.user = 'rapp'
        req.language = 'en'
        response = stt_service(req)
        words_basic = len(response.words)

        # Check number of words 
        self.assertEqual( words_basic, 0)

        # Check number of alternatives
        self.assertEqual( len(response.alternatives), 0) 

    def test_notExistentFile(self):
        rospack = rospkg.RosPack()
        google_service = rospy.get_param(\
                "rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_testing_tools') + \
                '/testing_tools/test_data/something.flac'
        req.audio_type = 'nao_wav_1_ch'
        req.user = 'rapp'
        req.language = 'en'
        response = stt_service(req)
        words_basic = len(response.words)

        # self.assertEqual(response.words, 1)

        # Check number of words 
        self.assertEqual( words_basic, 0)

        # Check number of alternatives
        self.assertEqual( len(response.alternatives), 0) 

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'SpeechToTextFunc', SpeechToTextFunc)














