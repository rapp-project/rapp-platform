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
import timeit
import unittest
import rospkg

__path__ = os.path.dirname(os.path.realpath(__file__))

from RappCloud import RappPlatformAPI

class TextToSpeechTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()

    def test_greek_test(self):
        destFilePath = '/tmp/ttsClient.wav'

        response = self.ch.textToSpeech('Καλησπέρα. Είμαι ο ναο', 'el', destFilePath)
        
        self.assertEqual(response['error'], u'')
        self.assertEqual(os.path.getsize(destFilePath) > 0, True)

    def test_tts_erroneous(self):
        destFilePath = '/tmp/ttsClient.wav'
        
        response = self.ch.textToSpeech(3, 'el', destFilePath)
        self.assertNotEqual(response['error'], u'')
        response = self.ch.textToSpeech([], 'el', destFilePath)
        self.assertNotEqual(response['error'], u'')
        #response = self.ch.textToSpeech('Καλησπέρα. Είμαι ο ναο', '', destFilePath)
        #self.assertNotEqual(response['error'], u'')
        response = self.ch.textToSpeech('Καλησπέρα. Είμαι ο ναο', 3, destFilePath)
        self.assertNotEqual(response['error'], u'')
        response = self.ch.textToSpeech('Καλησπέρα. Είμαι ο ναο', [], destFilePath)
        self.assertNotEqual(response['error'], u'')
        #response = self.ch.textToSpeech('Καλησπέρα. Είμαι ο ναο', 'el', '')
        #self.assertNotEqual(response['error'], u'')
        response = self.ch.textToSpeech('Καλησπέρα. Είμαι ο ναο', 'el', [])
        self.assertNotEqual(response['error'], u'')
        response = self.ch.textToSpeech('Καλησπέρα. Είμαι ο ναο', 'el', 3)
        self.assertNotEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
