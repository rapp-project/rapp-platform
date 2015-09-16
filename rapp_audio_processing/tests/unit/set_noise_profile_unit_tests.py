#! /usr/bin/env python

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
        self.auxiliary_files_url = rospack.get_path("rapp_auxiliary_files")
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
             
