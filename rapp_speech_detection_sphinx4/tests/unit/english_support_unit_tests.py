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
        conf = self.english_support_module.getLimitedVocebularyConfiguration(\
                ['this', 'is', 'a', 'test'],\
                [],\
                [])
        
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
        conf = self.english_support_module.getLimitedVocebularyConfiguration(\
                ['kakakakaka', 'lslslslsl', 'a', 'test'],\
                [],\
                [])
        
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
