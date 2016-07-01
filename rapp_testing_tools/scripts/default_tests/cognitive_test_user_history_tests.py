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

__path__ = os.path.dirname(os.path.realpath(__file__))

from RappCloud import RappPlatformAPI

class CognitiveTestHistoryTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()

    def test_historyArithmetic(self):
        response = self.ch.cognitiveGetHistory('ArithmeticCts', 0, 100000000)
        self.assertEqual(response['error'], u'')
        self.assertNotEqual(len(response['records']), 0)

    def test_historyReasoning(self):
        response = self.ch.cognitiveGetHistory('ReasoningCts', 0, 100000000)
        self.assertEqual(response['error'], u'')
        self.assertNotEqual(len(response['records']), 0)

    def test_historyAwareness(self):
        response = self.ch.cognitiveGetHistory('AwarenessCts', 0, 100000000)
        self.assertEqual(response['error'], u'')
        self.assertNotEqual(len(response['records']), 0)

    def test_historyAll(self):
        response = self.ch.cognitiveGetHistory('', 0, 100000000)
        self.assertEqual(response['error'], u'')
        self.assertNotEqual(len(response['records']), 0)

    def test_history_wrongType(self):
        response = self.ch.cognitiveGetHistory('T', 0, 100000000)
        self.assertNotEqual(response['error'], u'')
 
    def test_history_wrongTypeType(self):
        response = self.ch.cognitiveGetHistory(0, 0, 100000000)
        self.assertNotEqual(response['error'], u'')
    
    def test_history_wrongFromType(self):
        response = self.ch.cognitiveGetHistory('', '0', 100000000)
        self.assertNotEqual(response['error'], u'')
 
    def test_history_wrongToType(self):
        response = self.ch.cognitiveGetHistory('T', 0, '100000000')
        self.assertNotEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
