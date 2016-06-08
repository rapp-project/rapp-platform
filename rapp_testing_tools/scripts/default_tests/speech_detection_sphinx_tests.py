#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# contact: klpanagi@gmail.com, etsardou@iti.gr

import os
from os import path
import timeit
import unittest
import rospkg

__path__ = os.path.dirname(os.path.realpath(__file__))

from RappCloud import RappPlatformAPI

class SpeechDetectionSphinxTests(unittest.TestCase):

    def setUp(self):
        rospack = rospkg.RosPack()
        self.pkgDir = rospack.get_path('rapp_testing_tools')
 
        self.ch = RappPlatformAPI()

    def test_ogg_oxi(self):
        audioFile = path.join(self.pkgDir, 'test_data',
            'speech_detection_samples', 'recording_oxi.ogg')
        
        response = self.ch.speechRecognitionSphinx(audioFile, 'nao_ogg', 'el',\
            [u'ναι', u'οχι'])

        valid_words_found = [u'οχι']

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['words'], valid_words_found)

    def test_ogg_no(self):
        audioFile = path.join(self.pkgDir, 'test_data',
            'speech_detection_samples', 'recording_no.ogg')
        
        response = self.ch.speechRecognitionSphinx(audioFile, 'nao_ogg', 'en',\
            [u'yes', u'no'])

        valid_words_found = [u'no']

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['words'], valid_words_found)

    def test_wav_1_ch_yes_no(self):
        audioFile = path.join(self.pkgDir, 'test_data',
            'yes-no.wav')
        
        response = self.ch.speechRecognitionSphinx(audioFile, 'nao_wav_1_ch', 'en',\
            [u'yes', u'no'])

        valid_words_found = [u'yes', u'no']

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['words'], valid_words_found)

    def test_wav_1_ch_nai_oxi(self):
        audioFile = path.join(self.pkgDir, 'test_data',
            'nai-oxi-test.wav')
        
        response = self.ch.speechRecognitionSphinx(audioFile, 'nao_wav_1_ch', 'el',\
            [u'ναι', u'οχι', u'ισως'])

        valid_words_found = [u'ναι', u'οχι', u'ισως']

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['words'], valid_words_found)

    def test_headset_nai_oxi(self):
        audioFile = path.join(self.pkgDir, 'test_data',
            'microphone_nai.wav')
        
        response = self.ch.speechRecognitionSphinx(audioFile, 'headset', 'el',\
            [u'ναι', u'οχι'])

        valid_words_found = [u'ναι']

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['words'], valid_words_found)

    def test_speech_erroneous(self):
        audioFile = path.join(self.pkgDir, 'test_data', 'microphone_nai.wav')

        response = self.ch.speechRecognitionSphinx('', 'headset', 'el', [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx([], 'headset', 'el', [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(3, 'headset', 'el', [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, '', 'el', [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, [], 'el', [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, 3, 'el', [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, 'headset', '', [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, 'headset', [], [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, 'headset', 3, [u'ναι', u'οχι'])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, 'headset', 'el', 3)
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionSphinx(audioFile, 'headset', 'el', '')
        self.assertNotEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
