#! /usr/bin/env python

import sys
import unittest
import roslib
import rospkg
import os

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import TransformAudio

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        rospack = rospkg.RosPack()
        self.auxiliary_files_url = rospack.get_path("rapp_testing_tools") + \
                '/testing_tools/test_data'
        self.transform_audio_module = TransformAudio()

    def tearDown(self):
        self.transform_audio_module = None
        self.rospack = None

    def test_wavToFlac(self):
        source_name = self.auxiliary_files_url + "/nai_sample.wav"
        source_type = 'nao_wav_1_ch'

        target_type = 'flac'
        target_name = self.auxiliary_files_url + "/nai_sample.flac"
        target_channels = 1
        target_rate = 16000

        result, final_name = self.transform_audio_module.transform_audio( \
                source_type, source_name, target_type, target_name, \
                target_channels, target_rate )

        self.assertEqual( result, 'success' )

        transformedExists = os.path.isfile( target_name )
        print target_name
        #self.assertTrue( transformedExists )
        self.assertEqual(transformedExists, True)


