#! /usr/bin/env python
# -*- encode: utf-8 -*-

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

import sys
import unittest
import roslib
import rospkg
import os

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import SetNoiseProfile

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        rospack = rospkg.RosPack()
        self.auxiliary_files_url = rospack.get_path("rapp_testing_tools") +\
                '/test_data'
        self.module = SetNoiseProfile()

    def tearDown(self):
        self.module = None
        self.rospack = None

    def test_ogg(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_ogg_d05_a1.ogg',\
                'nao_ogg')
        self.assertEqual(result, 'true')

        # Check that the file exists
        path = os.path.expanduser(\
                '~/rapp_platform_files/audio_processing/rapp/noise_profile/noise_profile_nao_ogg')
        self.assertEqual(\
            os.path.isfile(\
            path),\
            True)

    def test_wav_1_ch(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_sample.wav',\
                'nao_wav_1_ch')
        self.assertEqual(result, 'true')

        # Check that the file exists
        self.assertEqual(\
            os.path.isfile(\
            os.path.expanduser(\
            '~/rapp_platform_files/audio_processing/rapp/noise_profile/noise_profile_nao_wav_1_ch')),\
            True)

    def test_wav_4_ch(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_wav_d05_a1.wav',\
                'nao_wav_4_ch')
        self.assertEqual(result, 'true')

        # Check that the file exists
        self.assertEqual(\
            os.path.isfile(\
            os.path.expanduser(\
            '~/rapp_platform_files/audio_processing/rapp/noise_profile/noise_profile_nao_wav_4_ch')),\
            True)

    def test_wav_6_ch(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_wav_d05_a1.wav',\
                'nao_wav_6_ch')
        self.assertNotEqual(result, 'true')

    def test_no_silence_file(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_aoua.wav',\
                'nao_wav_1_ch')
        self.assertNotEqual(result, 'true')

    def test_wav_1_ch_with_ogg_type(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_sample.wav',\
                'nao_ogg')
        self.assertEqual("Error" in result, True)

    def test_wav_1_ch_with_wav_4_ch_type(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_sample.wav',\
                'nao_wav_4_ch')
        self.assertEqual("Error" in result, True)

    def test_wav_4_ch_with_ogg_type(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_wav_d05_a1.wav',\
                'nao_ogg')
        self.assertEqual("Error" in result, True)

    def test_wav_4_ch_with_wav_1_ch_type(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_wav_d05_a1.wav',\
                'nao_wav_1_ch')
        self.assertEqual("Error" in result, True)

    def test_ogg_with_wav_1_ch_type(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_ogg_d05_a1.ogg',\
                'nao_wav_1_ch')
        self.assertEqual("Error" in result, True)

    def test_ogg_with_wav_4_ch_type(self):
        result = self.module.setNoise_profile(\
                'rapp',\
                self.auxiliary_files_url + '/silence_ogg_d05_a1.ogg',\
                'nao_wav_4_ch')
        self.assertEqual("Error" in result, True)

