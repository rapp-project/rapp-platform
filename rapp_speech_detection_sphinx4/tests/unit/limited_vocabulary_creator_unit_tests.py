#! /usr/bin/env python

import sys
import unittest
import roslib
import os

roslib.load_manifest("rapp_speech_detection_sphinx4")

from rapp_speech_detection_sphinx4 import LimitedVocabularyCreator

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        self.module = LimitedVocabularyCreator() 
    
    def tearDown(self):
        self.module = None

    def test_normalCase(self):
        # Prepare data
        words = {}
        words['autos'] = ['AA','F','T','OW','S']
        words['dyskolo'] = ['D','IH','S','K','OW','L','OW']
        grammar = ['autos', 'dyskolo', 'autos dyskolo']
        sentences = ['autos', 'dyskolo']

        conf = self.module.createConfigurationFiles(words, grammar, sentences)
        
        # Check if files are there
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

        # Check data in files
        with open(dictionary) as f:
            dict_words = f.readlines()
        self.assertEqual(dict_words[0], 'autos AA F T OW S\n')
        self.assertEqual(dict_words[1], 'dyskolo D IH S K OW L OW\n')

        # language model is the lm.dmp. Check the .txt instead.
        language_model = language_model[:-6] + 'txt'
        with open(language_model) as f:
            sentences = f.readlines()
        self.assertEqual(sentences[0], '<s> autos </s>\n')
        self.assertEqual(sentences[1], '<s> dyskolo </s>\n')

        # Check grammar
        grammar = grammar_folder + 'custom.gram'
        with open(grammar) as f:
            grams = f.readlines()
        self.assertEqual(grams[2], 'public <cmd1>=autos;\n')
        self.assertEqual(grams[3], 'public <cmd2>=dyskolo;\n')
        self.assertEqual(grams[4], 'public <cmd3>=autos dyskolo;\n')


    def test_noSentences(self):
        # Prepare data
        words = {}
        words['autos'] = ['AA','F','T','OW','S']
        words['dyskolo'] = ['D','IH','S','K','OW','L','OW']
        grammar = []
        sentences = []

        conf = self.module.createConfigurationFiles(words, grammar, sentences)
        
        # Check if files are there
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

        # Check data in files
        with open(dictionary) as f:
            dict_words = f.readlines()
        self.assertEqual(dict_words[0], 'autos AA F T OW S\n')
        self.assertEqual(dict_words[1], 'dyskolo D IH S K OW L OW\n')

