#! /usr/bin/env python
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

import sys
import unittest
import roslib
import os

roslib.load_manifest("rapp_speech_detection_sphinx4")

from rapp_speech_detection_sphinx4 import GreekSupport

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        self.module = GreekSupport()

    def tearDown(self):
        self.module = None

    def test_transformWords_case_autos(self):
        words =  ['αυτός']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["auto's'"], words[0])
        self.assertEqual(enh["auto's'"], ['AA','F','T','OW','S'])

    def test_transformWords_case_mallon(self):
        words =  ['μάλλον']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["ma'llon"], words[0])
        self.assertEqual(enh["ma'llon"], ['M','AA','L','OW','N'])

    def test_transformWords_case_euxaristw(self):
        words =  ['ευχαριστώ']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["euxaristw'"], words[0])
        self.assertEqual(enh["euxaristw'"], ['EH','F','HH','AA','R','IH','S','T','OW'])

    def test_transformWords_case_poini(self):
        words =  ['ποινή']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["poinh'"], words[0])
        self.assertEqual(enh["poinh'"], ['P','IH','N','IH'])

    def test_transformWords_case_diairesi(self):
        words =  ['διαίρεση']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["diai'resh"], words[0])
        self.assertEqual(enh["diai'resh"], ['DH','IH','EH','R','EH','S','IH'])

    def test_transformWords_case_exairetika(self):
        words =  ['εξαιρετικά']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["eksairetika'"], words[0])
        self.assertEqual(enh["eksairetika'"], ['EH','K','S','EH','R','EH','T','IH','K','AA'])

    def test_transformWords_case_efstoxos(self):
        words =  ['εύστοχος']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["eu'stoxos'"], words[0])
        self.assertEqual(enh["eu'stoxos'"], ['EH','F','S','T','OW','HH','OW','S'])

    def test_transformWords_case_kalytereyw(self):
        words =  ['καλυτερεύω']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["kalutereu'w"], words[0])
        self.assertEqual(enh["kalutereu'w"], ['K','AA','L','IH','T','EH','R','EH','V','OW'])

    def test_transformWords_case_wriaios(self):
        words =  ['Ωριαίος']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["wriai'os'"], words[0])
        self.assertEqual(enh["wriai'os'"], ['OW','R','IH','EH','OW','S'])

    def test_transformWords_case_xasma(self):
        words =  ['Χάσμα']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["xa'sma"], words[0])
        self.assertEqual(enh["xa'sma"], ['HH','AA','Z','M','AA'])

    def test_transformWords_case_augo(self):
        words =  ['Αυγό']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["augo'"], words[0])
        self.assertEqual(enh["augo'"], ['AA','V','W','OW'])

    def test_transformWords_case_epistimonas(self):
        words =  ['επιστήμονας']
        [enh, engl] = self.module._transformWords(words)

        self.assertEqual(engl["episth'monas'"], words[0])
        self.assertEqual(enh["episth'monas'"], ['EH','P','IH','S','T','IH','M','OW','N','AA','S'])

