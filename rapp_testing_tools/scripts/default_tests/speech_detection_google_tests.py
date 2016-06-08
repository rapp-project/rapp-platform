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

class SppechDetectionGoogleTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()

        rospack = rospkg.RosPack()
        self.pkgDir = rospack.get_path('rapp_testing_tools')
 
    def test_speechNormal(self):
        audioFile = path.join(self.pkgDir, 'test_data',
            'speech_detection_samples', 'recording_sentence1.ogg')

        valid_words_found = ['I', 'want', 'to', 'go', 'out']

        response = self.ch.speechRecognitionGoogle(audioFile, 'nao_ogg', 'en')

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['words'], valid_words_found)
 
    def test_erroneous(self):
        audioFile = path.join(self.pkgDir, 'test_data',
            'speech_detection_samples', 'recording_sentence1.ogg')

        response = self.ch.speechRecognitionGoogle('', 'nao_ogg', 'en')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(3, 'nao_ogg', 'en')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle([], 'nao_ogg', 'en')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(audioFile, '', 'en')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(audioFile, 3, 'en')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(audioFile, [], 'en')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(audioFile, 'nao_ogg', 'klklklkl')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(audioFile, 'nao_ogg', '')
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(audioFile, 'nao_ogg', 3)
        self.assertNotEqual(response['error'], u'')
        response = self.ch.speechRecognitionGoogle(audioFile, 'nao_ogg', [])
        self.assertNotEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
