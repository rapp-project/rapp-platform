#! /usr/bin/env python

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

import sys
import unittest
import roslib
import os

roslib.load_manifest("rapp_speech_detection_sphinx4")

from rapp_speech_detection_sphinx4 import EnglishSupport
from rapp_exceptions import RappError

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        self.english_support_module = EnglishSupport()

    def tearDown(self):
        self.utilities_module = None

    def test_genericConfigurationFiles(self):
        conf = self.english_support_module.getGenericConfiguration()

        jar_path = conf['jar_path']
        jar_path = jar_path.split(':')
        self.assertEqual(jar_path[0], '.')
        self.assertEqual(os.path.isfile(jar_path[1]), True)
        self.assertEqual(os.path.isdir(jar_path[2]), True)

        conf_path = conf['configuration_path']
        self.assertEqual(os.path.isfile(conf_path), True)

        acoustic = conf['acoustic_model']
        self.assertEqual(os.path.isdir(acoustic), True)

        grammar_folder = conf['grammar_folder']
        self.assertEqual(os.path.isdir(grammar_folder), True)

        dictionary = conf['dictionary']
        self.assertEqual(os.path.isfile(dictionary), True)

        language_model = conf['language_model']
        self.assertEqual(os.path.isfile(language_model), True)

    def test_limitedVocabularyConfigurationFiles_correctCase(self):
        try:
            [conf, wordDict] = self.english_support_module.getLimitedVocebularyConfiguration(\
                    ['this', 'is', 'a', 'test'],\
                    [],\
                    [])
        except RappError as e:
            self.assertEqual(e.value, True)

        jar_path = conf['jar_path']
        jar_path = jar_path.split(':')
        self.assertEqual(jar_path[0], '.')
        self.assertEqual(os.path.isfile(jar_path[1]), True)
        self.assertEqual(os.path.isdir(jar_path[2]), True)

        conf_path = conf['configuration_path']
        self.assertEqual(os.path.isfile(conf_path), True)

        acoustic = conf['acoustic_model']
        self.assertEqual(os.path.isdir(acoustic), True)

        grammar_folder = conf['grammar_folder']
        self.assertEqual(os.path.isdir(grammar_folder), True)

        dictionary = conf['dictionary']
        self.assertEqual(os.path.isfile(dictionary), True)

        language_model = conf['language_model']
        self.assertEqual(os.path.isfile(language_model), True)

    def test_limitedVocabularyConfigurationFiles_notExistentWords(self):
        # This produces the proper files without the erroneous words
        try:
            [conf, success] = self.english_support_module.getLimitedVocebularyConfiguration(\
                    ['kakakakaka', 'lslslslsl', 'a', 'test'],\
                    [],\
                    [])
        except RappError as e:
            self.assertNotEqual(e.value, '')
            self.assertNotEqual(e.value, True)

