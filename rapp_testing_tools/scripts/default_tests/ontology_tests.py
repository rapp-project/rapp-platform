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

class OntologyTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()

    def test_superclass_of(self):
        valid_results = [ u'http://knowrob.org/kb/knowrob.owl#Box-Container',\
            u'http://knowrob.org/kb/knowrob.owl#FurniturePiece', \
            u'http://knowrob.org/kb/knowrob.owl#HeatingDevice', \
            u'http://knowrob.org/kb/knowrob.owl#HouseholdAppliance']

        response = self.ch.ontologySuperclasses('Oven')

        self.assertEqual(response['results'], valid_results)
        self.assertEqual(response['error'], u'')

    def test_subclass_of(self):
        valid_results = [ 
            'http://knowrob.org/kb/knowrob.owl#MicrowaveOven', \
            'http://knowrob.org/kb/knowrob.owl#RegularOven', \
            'http://knowrob.org/kb/knowrob.owl#ToasterOven'\
            ]
 
        response = self.ch.ontologySubclasses('Oven')

        self.assertEqual(response['results'], valid_results)
        self.assertEqual(response['error'], u'')

    def test_is_subsuperclass_of(self):
        response = self.ch.ontologyIsSubsuperclass('Oven', 'MicrowaveOven', True)

        self.assertEqual(response['result'], True)
        self.assertEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
