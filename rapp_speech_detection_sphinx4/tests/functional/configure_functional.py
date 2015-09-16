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
  SpeechRecognitionSphinx4TotalSrvResponse
  )

class SpeechDetSphinx4Func(unittest.TestCase):

    def test_configureService(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/testing_tools/test_data'

        conf_service = rospy.get_param(\
                "rapp_speech_detection_sphinx4_configuration_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, SpeechRecognitionSphinx4ConfigureSrv)
        
        req = SpeechRecognitionSphinx4ConfigureSrvRequest()
        req.language = 'en'
        req.words = ['hello', 'this']
        req.grammar = req.words + ['hello this']
        req.sentences = req.words
        response = test_service(req)
        self.assertEqual( response.error, '' )

    def test_configureService_stress(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/testing_tools/test_data'

        conf_service = rospy.get_param(\
                "rapp_speech_detection_sphinx4_configuration_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, SpeechRecognitionSphinx4ConfigureSrv)
        
        for i in range(0,40):
            req = SpeechRecognitionSphinx4ConfigureSrvRequest()
            req.language = 'en'
            req.words = ['hello', 'this']
            req.grammar = req.words + ['hello this']
            req.sentences = req.words
            response = test_service(req)
            self.assertEqual( response.error, '' )


    def test_configureServiceErrorLanguage(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/testing_tools/test_data'

        conf_service = rospy.get_param(\
                "rapp_speech_detection_sphinx4_configuration_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, SpeechRecognitionSphinx4ConfigureSrv)
        
        req = SpeechRecognitionSphinx4ConfigureSrvRequest()
        req.language = 'es'
        req.words = ['hello', 'this']
        req.grammar = req.words + ['hello this']
        req.sentences = req.words
        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_configureServiceErrorSentences(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/testing_tools/test_data'

        conf_service = rospy.get_param(\
                "rapp_speech_detection_sphinx4_configuration_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, SpeechRecognitionSphinx4ConfigureSrv)
        
        req = SpeechRecognitionSphinx4ConfigureSrvRequest()
        req.language = 'en'
        req.words = ['hello', 'this']
        req.grammar = req.words + ['hello this']
        req.sentences = ['hello', 'test']
        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_configureServiceErrorGrammar(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/testing_tools/test_data'

        conf_service = rospy.get_param(\
                "rapp_speech_detection_sphinx4_configuration_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, SpeechRecognitionSphinx4ConfigureSrv)
        
        req = SpeechRecognitionSphinx4ConfigureSrvRequest()
        req.language = 'en'
        req.words = ['hello', 'this']
        req.grammar = req.words + ['hello that']
        req.sentences = ['hello', 'this']
        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_configureServiceNotExistentWords(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/testing_tools/test_data'

        conf_service = rospy.get_param(\
                "rapp_speech_detection_sphinx4_configuration_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, SpeechRecognitionSphinx4ConfigureSrv)
        
        req = SpeechRecognitionSphinx4ConfigureSrvRequest()
        req.language = 'en'
        req.words = ['hello', 'kakakakaka']
        req.grammar = []
        req.sentences = []
        response = test_service(req)
        self.assertNotEqual( response.error, '' )

    def test_configureServiceGreekWords(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/testing_tools/test_data'

        conf_service = rospy.get_param(\
                "rapp_speech_detection_sphinx4_configuration_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, SpeechRecognitionSphinx4ConfigureSrv)
        
        req = SpeechRecognitionSphinx4ConfigureSrvRequest()
        req.language = 'gr'
        req.words = ['αθηνά', 'έλεγχος']
        req.grammar = []
        req.sentences = []
        response = test_service(req)
        self.assertEqual( response.error, '' )

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'SpeechDetSphinx4Func', SpeechDetSphinx4Func)

