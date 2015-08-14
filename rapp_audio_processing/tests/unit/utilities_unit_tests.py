#! /usr/bin/env python

import sys
import unittest
import roslib
import os

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import Utilities

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        self.utilities_module = Utilities() 
    
    def tearDown(self):
        self.utilities_module = None

    def test_existentFiles(self):
        # Create the files
        files = []
        files.append("/tmp/file_1")
        files.append("/tmp/file_2")
        for f in files:
            open(f, 'w')
        
        result = self.utilities_module.cleanup(files)
        # Check if it thinks it erased them
        self.assertEqual(result, True)

        for f in files:
            result = os.path.isfile(f)
            self.assertNotEqual(result, True)

    def test_notExistentFiles(self):
        # Create the files
        files = []
        files.append("/tmp/file_1")
        files.append("/tmp/file_2")
       
        result = self.utilities_module.cleanup(files)
        # Check if it thinks it erased them
        self.assertNotEqual(result, True)

    def test_mixedCases(self):
        # Create the files
        files = []
        files.append("/tmp/file_1")
        files.append("/tmp/file_2")
        open(files[0], 'w')

        result = self.utilities_module.cleanup(files)
        # Check if it thinks it erased them
        self.assertNotEqual(result, True)

