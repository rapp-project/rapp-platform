#! /usr/bin/env python

import sys
import unittest
import roslib
import os

roslib.load_manifest("rapp_speech_detection_sphinx4")

from rapp_speech_detection_sphinx4 import LimitedVocabularyCreator
from rapp_speech_detection_sphinx4 import RappError

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
        
        try:
            conf = self.module.createConfigurationFiles(words, grammar, sentences)
        except RappError as e:
            self.assertEqual(e.value, True)
            
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
        self.assertEqual('autos AA F T OW S\n' in dict_words, True)
        self.assertEqual('dyskolo D IH S K OW L OW\n' in dict_words, True)

        # Check lm.dmp and .arpa
        lm = language_model
        self.assertEqual(os.path.isfile(lm), True)
        lm = language_model[:-6] + 'arpa'
        self.assertEqual(os.path.isfile(lm), True)

        # language model is the lm.dmp. Check the .txt instead.
        language_model = language_model[:-6] + 'txt'
        with open(language_model) as f:
            sentences = f.readlines()
        self.assertEqual('<s> autos </s>\n' in sentences, True)
        self.assertEqual('<s> dyskolo </s>\n' in sentences, True)

        # Check grammar
        grammar = os.path.join( grammar_folder, 'custom.gram' )
        with open(grammar) as f:
            grams = f.readlines()
        self.assertEqual('public <cmd1>="autos";\n' in grams, True)
        self.assertEqual('public <cmd2>="dyskolo";\n' in grams, True)
        self.assertEqual('public <cmd3>="autos dyskolo";\n' in grams, True)


    def test_noSentences(self):
        # Prepare data
        words = {}
        words['ekei'] = ['EH','K','IH']
        words['kserw'] = ['K','S','EH','R','OW']
        grammar = []
        sentences = []

        try:
            conf = self.module.createConfigurationFiles(words, grammar, sentences)
        except RappError as e:
            self.assertEqual(e.value, True)

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
        self.assertEqual('ekei EH K IH\n' in dict_words, True)
        self.assertEqual('kserw K S EH R OW\n' in dict_words, True)

        # Check lm.dmp and .arpa
        lm = language_model
        self.assertEqual(os.path.isfile(lm), True)
        lm = language_model[:-6] + 'arpa'
        self.assertEqual(os.path.isfile(lm), True)

        # language model is the lm.dmp. Check the .txt instead.
        language_model = language_model[:-6] + 'txt'
        with open(language_model) as f:
            sentences = f.readlines()
        self.assertEqual('<s> ekei </s>\n' in sentences, True)
        self.assertEqual('<s> kserw </s>\n' in sentences, True)

