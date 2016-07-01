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
import sys
import timeit
import time
import unittest

__path__ = os.path.dirname(os.path.realpath(__file__))

from RappCloud import RappPlatformAPI

class CognitiveTestChooserTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()
        # Wait till the service is up using an erroneous call for fast return
        counter = 0
        timeout = 120
        while counter < timeout:
            counter = counter + 1
            res = self.ch.cognitiveExerciseSelect(0)
            if res['error'] != u'Connection Error':
                break
            time.sleep(1)
            print 'Waiting for service... ' + str(timeout - counter)

        if counter == timeout:
            self.assertEqual('Connection error', True)

    def test_selectArithmeticTest(self):
        res = self.ch.cognitiveExerciseSelect('ArithmeticCts', 'TransactionChangeCts', '1', '1')
        self.assertEqual(res['error'], u"")
        self.assertNotEqual(res['test_instance'], u"")
        self.assertNotEqual(res['test_type'], u"")
        self.assertNotEqual(res['test_subtype'], u"")
        self.assertNotEqual(res['questions'], u"")
        self.assertNotEqual(res['possib_ans'], u"")
        self.assertNotEqual(res['correct_ans'], u"")

    def test_selectArithmeticTest_default_params(self):
        res = self.ch.cognitiveExerciseSelect('ArithmeticCts')
        self.assertEqual(res['error'], u"")
        self.assertNotEqual(res['test_instance'], u"")
        self.assertNotEqual(res['test_type'], u"")
        self.assertNotEqual(res['test_subtype'], u"")
        self.assertNotEqual(res['questions'], u"")
        self.assertNotEqual(res['possib_ans'], u"")
        self.assertNotEqual(res['correct_ans'], u"")

    def test_selectAwarenessTest(self):
        res = self.ch.cognitiveExerciseSelect('AwarenessCts')
        self.assertEqual(res['error'], u"")
        self.assertNotEqual(res['test_instance'], u"")
        self.assertNotEqual(res['test_type'], u"")
        self.assertNotEqual(res['test_subtype'], u"")
        self.assertNotEqual(res['questions'], u"")
        self.assertNotEqual(res['possib_ans'], u"")
        self.assertNotEqual(res['correct_ans'], u"")

    def test_selectReasoningTest(self):
        res = self.ch.cognitiveExerciseSelect('ReasoningCts')
        self.assertEqual(res['error'], u"")
        self.assertNotEqual(res['test_instance'], u"")
        self.assertNotEqual(res['test_type'], u"")
        self.assertNotEqual(res['test_subtype'], u"")
        self.assertNotEqual(res['questions'], u"")
        self.assertNotEqual(res['possib_ans'], u"")
        self.assertNotEqual(res['correct_ans'], u"")

    def test_selectNotExistentTest(self):
        res = self.ch.cognitiveExerciseSelect('KoukouroukouCts')
        self.assertNotEqual(res['error'], u"")

    def test_selectWrongType_TestType(self):
        res = self.ch.cognitiveExerciseSelect(3)
        self.assertNotEqual(res['error'], u"")

    def test_selectWrongType_TestSubtype(self):
        res = self.ch.cognitiveExerciseSelect('ArithmeticCts', 3)
        self.assertNotEqual(res['error'], u"")

    def test_selectWrongType_TestDiff(self):
        res = self.ch.cognitiveExerciseSelect('ArithmeticCts', '', 4)
        self.assertNotEqual(res['error'], u"")

    def test_selectWrongType_TestIndex(self):
        res = self.ch.cognitiveExerciseSelect('ArithmeticCts', '', '1', 3)
        self.assertNotEqual(res['error'], u"")

if __name__ == "__main__":
    unittest.main()
