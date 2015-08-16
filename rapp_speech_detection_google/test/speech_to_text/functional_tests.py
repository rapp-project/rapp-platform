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
        google_service = rospy.get_param("rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_auxiliary_files') + '/nai_sample.wav'
        response = stt_service(req)
        words_basic = len(response.words)

        # self.assertEqual(response.words, 1)

        # Check number of words 
        self.assertEqual( words_basic, 0)

        # Check number of alternatives
        self.assertEqual( len(response.alternatives), 0) 

    def test_imageFile(self):
        rospack = rospkg.RosPack()
        google_service = rospy.get_param("rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_auxiliary_files') + '/Lenna.png'
        response = stt_service(req)
        words_basic = len(response.words)

        # self.assertEqual(response.words, 1)

        # Check number of words 
        self.assertEqual( words_basic, 0)

        # Check number of alternatives
        self.assertEqual( len(response.alternatives), 0) 

    def test_notExistentFile(self):
        rospack = rospkg.RosPack()
        google_service = rospy.get_param("rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_auxiliary_files') + '/something.flac'
        response = stt_service(req)
        words_basic = len(response.words)

        # self.assertEqual(response.words, 1)

        # Check number of words 
        self.assertEqual( words_basic, 0)

        # Check number of alternatives
        self.assertEqual( len(response.alternatives), 0) 


    def test_flacFile(self):
        rospack = rospkg.RosPack()
        google_service = rospy.get_param("rapp_speech_detection_google_detect_speech_topic")
        rospy.wait_for_service(google_service)
        stt_service = rospy.ServiceProxy(google_service, SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename = rospack.get_path('rapp_auxiliary_files') + '/test.flac'
        response = stt_service(req)
        words_basic = len(response.words)

        # Check number of words 
        self.assertEqual( words_basic, 6)

        # Check the actual words
        self.assertEqual( response.words[0], "I") 
        self.assertEqual( response.words[1], "want") 
        self.assertEqual( response.words[2], "to") 
        self.assertEqual( response.words[3], "use") 
        self.assertEqual( response.words[4], "the") 
        self.assertEqual( response.words[5], "Skype") 

        # Check number of alternatives
        self.assertEqual( len(response.alternatives), 4) 

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'SpeechToTextFunc', SpeechToTextFunc)














