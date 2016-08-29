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

class VisualLocalizationTests(unittest.TestCase):

    def setUp(self): 
        rospack = rospkg.RosPack()
        self.pkgDir = rospack.get_path('rapp_testing_tools')
        
        self.ch = RappPlatformAPI()

    def test_init(self):
        response = self.ch.visualLocalizationInit('')
        
        self.assertEqual(response['error'], u'')
        self.assertEqual(response['id'] == 0, True)

#    def test_erroneousPath(self):
#        imagepath = path.join(self.pkgDir, 'test_data', 'lamp_on.jpg')
#
#        response = self.ch.hazardDetectionLights(imagepath)
#        
#        self.assertNotEqual(response['error'], u'')

#    def test_wrongPathType(self):
#        imagepath = 3
#
#        response = self.ch.hazardDetectionLights(imagepath)
#        
#        self.assertNotEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
