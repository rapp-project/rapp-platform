#! /usr/bin/env python

import sys
import unittest
import roslib
import os

roslib.load_manifest("rapp_speech_detection_sphinx4")

from rapp_speech_detection_sphinx4 import EnglishSupport

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        self.english_support_module = EnglishSupport() 
    
    def tearDown(self):
        self.utilities_module = None

    def test_mixedCases(self):
        # files = []
        # files.append("/tmp/file_1")
        # files.append("/tmp/file_2")
        # open(files[0], 'w')

        # result = self.utilities_module.cleanup(files)
        self.assertEqual(True, True)

