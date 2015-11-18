#! /usr/bin/env python

import sys
import unittest
import roslib
import rospkg

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import DetectSilence

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        rospack = rospkg.RosPack()
        self.auxiliary_files_url = rospack.get_path("rapp_testing_tools") +\
                '/test_data'
        self.detect_silence_module = DetectSilence() 
    
    def tearDown(self):
        self.detect_silence_module = None
        self.rospack = None

    def test_silence(self):
        [sq, result] = self.detect_silence_module.detectSilence(\
                self.auxiliary_files_url + "/silence_sample.wav", 2.5)
        self.assertEqual(result, True)

    def test_noSilence(self):
        [sq, result] = self.detect_silence_module.detectSilence(\
                self.auxiliary_files_url + "/nai_sample.wav", 2.5)
        self.assertEqual(result, False)

    def test_notExistentFile(self):
        [sq, result] = self.detect_silence_module.detectSilence(\
                self.auxiliary_files_url + "/not_existent_file_sample.wav", 2.5)
        self.assertEqual(result, False)
        self.assertEqual(sq, -1)

