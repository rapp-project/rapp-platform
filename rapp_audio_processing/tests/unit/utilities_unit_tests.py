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

