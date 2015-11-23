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

# Authors: Manos Tsardoulias
# contact: etsardou@iti.gr

PKG='rapp_audio_processing'

import sys
import unittest
import rospy
import roslib
import rospkg
import os

from rapp_platform_ros_communications.srv import (
  AudioProcessingSetNoiseProfileSrv,
  AudioProcessingSetNoiseProfileSrvRequest
  )

class AudioProcessingSetNoiseProfileFunc(unittest.TestCase):

    def test_setNoiseProfileService_ogg(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
                "rapp_audio_processing_set_noise_profile_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, AudioProcessingSetNoiseProfileSrv)

        req = AudioProcessingSetNoiseProfileSrvRequest()
        req.audio_file_type = 'nao_ogg'
        req.noise_audio_file = aux + '/silence_ogg_d05_a1.ogg'
        req.user = 'rapp'
        response = test_service(req)
        self.assertEqual( response.error, '' )
        self.assertEqual( response.success, 'true' )

    def test_setNoiseProfileService_ogg_stress(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param(\
                "rapp_audio_processing_set_noise_profile_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(\
                conf_service, AudioProcessingSetNoiseProfileSrv)

        for i in range(0, 100):
            req = AudioProcessingSetNoiseProfileSrvRequest()
            req.audio_file_type = 'nao_ogg'
            req.noise_audio_file = aux + '/silence_ogg_d05_a1.ogg'
            req.user = 'rapp'
            response = test_service(req)
            self.assertEqual( response.error, '' )
            self.assertEqual( response.success, 'true' )

    def test_setNoiseProfileService_wav_1_ch(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param("rapp_audio_processing_set_noise_profile_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, AudioProcessingSetNoiseProfileSrv)

        req = AudioProcessingSetNoiseProfileSrvRequest()
        req.audio_file_type = 'nao_wav_1_ch'
        req.noise_audio_file = aux + '/silence_sample.wav'
        req.user = 'rapp'
        response = test_service(req)
        self.assertEqual( response.error, '' )
        self.assertEqual( response.success, 'true' )

    def test_setNoiseProfileService_wav_4_ch(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param("rapp_audio_processing_set_noise_profile_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, AudioProcessingSetNoiseProfileSrv)

        req = AudioProcessingSetNoiseProfileSrvRequest()
        req.audio_file_type = 'nao_wav_4_ch'
        req.noise_audio_file = aux + '/silence_wav_d05_a1.wav'
        req.user = 'rapp'
        response = test_service(req)
        self.assertEqual( response.error, '' )
        self.assertEqual( response.success, 'true' )

    def test_setNoiseProfileService_wav_6_ch(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param("rapp_audio_processing_set_noise_profile_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, AudioProcessingSetNoiseProfileSrv)

        req = AudioProcessingSetNoiseProfileSrvRequest()
        req.audio_file_type = 'nao_wav_6_ch'
        req.noise_audio_file = aux + '/silence_wav_d05_a1.wav'
        req.user = 'rapp'
        response = test_service(req)
        self.assertNotEqual( response.error, '' )
        self.assertEqual( response.success, 'false' )

    def test_setNoiseProfileService_no_silence_file(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param("rapp_audio_processing_set_noise_profile_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, AudioProcessingSetNoiseProfileSrv)

        req = AudioProcessingSetNoiseProfileSrvRequest()
        req.audio_file_type = 'nao_wav_4_ch'
        req.noise_audio_file = aux + '/silence_wav_d05_a1_nope.wav'
        req.user = 'rapp'
        response = test_service(req)
        self.assertNotEqual( response.error, '' )
        self.assertEqual( response.success, 'false' )

    def test_setNoiseProfileService_not_valid_user(self):
        rospack = rospkg.RosPack()
        aux = rospack.get_path('rapp_testing_tools') + '/test_data'

        conf_service = rospy.get_param("rapp_audio_processing_set_noise_profile_topic")
        rospy.wait_for_service(conf_service)
        test_service = rospy.ServiceProxy(conf_service, AudioProcessingSetNoiseProfileSrv)

        req = AudioProcessingSetNoiseProfileSrvRequest()
        req.audio_file_type = 'nao_wav_4_ch'
        req.noise_audio_file = aux + '/silence_wav_d05_a1.wav'
        req.user = 'not_existent_user'
        response = test_service(req)
        self.assertNotEqual( response.error, '' )
        self.assertEqual( response.success, 'false' )


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'AudioProcessingSetNoiseProfileFunc', AudioProcessingSetNoiseProfileFunc)

