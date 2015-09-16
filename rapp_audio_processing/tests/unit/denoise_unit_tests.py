#! /usr/bin/env python

import sys
import unittest
import roslib
import rospkg
import os
from pylab import *
from scipy.io import wavfile

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import SoxDenoise 

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        rospack = rospkg.RosPack()
        self.auxiliary_files_url = rospack.get_path("rapp_testing_tools") +\
                '/testing_tools/test_data'
        self.sox_denoise_module = SoxDenoise() 
    
    def tearDown(self):
        self.sox_denoise_module = None
        self.rospack = None

    def test_realFile(self):
        original_file = self.auxiliary_files_url + "/nai_sample.wav"
        denoised_file = self.auxiliary_files_url + "/nai_sample_sox_denoised.wav"
        user = 'rapp'
        audio_type = 'nao_wav_1_ch'
        scale = 0.2
    
        result = self.sox_denoise_module.soxDenoise(\
                user,\
                audio_type,\
                original_file,\
                denoised_file,\
                scale)
        # The function thinks the denoising succeded
        self.assertEqual(result, "true")
        # Check for the denoised file
        denoised_exists = os.path.isfile(denoised_file)
        self.assertEqual(denoised_exists, True)
        # Check if denoised energy is lower than the initial one
        samp_freq, signal_orig = wavfile.read(original_file)
        energy_orig = 0.0
        for i in signal_orig:
            energy_orig += i * 1.0 * i
        samp_freq, signal_denoised = wavfile.read(denoised_file)
        energy_denoised = 0.0
        for i in signal_denoised:
            energy_denoised += i * 1.0 * i
        self.assertGreater(energy_orig, energy_denoised)

        # erase the denoised file
        os.remove(denoised_file)

    def test_notExistentAudioFile(self):
        original_file = self.auxiliary_files_url + "/not_existent_sample.wav"
        denoised_file = self.auxiliary_files_url + "/nai_sample_sox_denoised.wav"
        user = 'rapp'
        audio_type = 'nao_wav_1_ch'
        scale = 0.2
    
        result = self.sox_denoise_module.soxDenoise(\
                user,\
                audio_type,\
                original_file,\
                denoised_file,\
                scale)
        # The function thinks the denoising succeded
        self.assertNotEqual(result, "true")

    def test_notExistentUser(self):
        original_file = self.auxiliary_files_url + "/nai_sample.wav"
        denoised_file = self.auxiliary_files_url + "/nai_sample_sox_denoised.wav"
        user = 'panagiotis'
        audio_type = 'nao_wav_1_ch'
        scale = 0.2
    
        result = self.sox_denoise_module.soxDenoise(\
                user,\
                audio_type,\
                original_file,\
                denoised_file,\
                scale)
        # The function thinks the denoising succeded
        self.assertNotEqual(result, "true")

    def test_notExistentAudioType(self):
        original_file = self.auxiliary_files_url + "/nai_sample.wav"
        denoised_file = self.auxiliary_files_url + "/nai_sample_sox_denoised.wav"
        user = 'rapp'
        audio_type = 'nao_wav_7_ch'
        scale = 0.2
    
        result = self.sox_denoise_module.soxDenoise(\
                user,\
                audio_type,\
                original_file,\
                denoised_file,\
                scale)
        # The function thinks the denoising succeded
        self.assertNotEqual(result, "true")

    def test_negativeScale(self):
        original_file = self.auxiliary_files_url + "/nai_sample.wav"
        denoised_file = self.auxiliary_files_url + "/nai_sample_sox_denoised.wav"
        user = 'rapp'
        audio_type = 'nao_wav_1_ch'
        scale = -0.2
    
        result = self.sox_denoise_module.soxDenoise(\
                user,\
                audio_type,\
                original_file,\
                denoised_file,\
                scale)
        # The function thinks the denoising succeded
        self.assertNotEqual(result, "true")

