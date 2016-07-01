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

class CognitiveTestScoresTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()

    def test_scoresArithmetic(self):
        response = self.ch.cognitiveGetScores('ArithmeticCts', 100000000)
        self.assertEqual(response['error'], u'')
        self.assertEqual(len(response['scores']), 1)
        self.assertEqual(len(response['test_classes']), 1)

    def test_scoresAwareness(self):
        response = self.ch.cognitiveGetScores('AwarenessCts', 100000000)
        self.assertEqual(response['error'], u'')
        self.assertEqual(len(response['scores']), 1)
        self.assertEqual(len(response['test_classes']), 1)

    def test_scoresReasoning(self):
        response = self.ch.cognitiveGetScores('ReasoningCts', 100000000)
        self.assertEqual(response['error'], u'')
        self.assertEqual(len(response['scores']), 1)
        self.assertEqual(len(response['test_classes']), 1)

    def test_scoresAll(self):
        response = self.ch.cognitiveGetScores('', 100000000)
        self.assertEqual(response['error'], u'')
        self.assertEqual(len(response['scores']) > 0, True)
        self.assertEqual(len(response['test_classes']) > 0, True)

    def test_scoresWrongType(self):
        response = self.ch.cognitiveGetScores(3, 100000000)
        self.assertNotEqual(response['error'], u'')

    def test_scoresWrongTypeList(self):
        response = self.ch.cognitiveGetScores([], 100000000)
        self.assertNotEqual(response['error'], u'')

    def test_scoresWrongType_to(self):
        response = self.ch.cognitiveGetScores('', '100000000')
        self.assertNotEqual(response['error'], u'')



if __name__ == "__main__":
    unittest.main()
