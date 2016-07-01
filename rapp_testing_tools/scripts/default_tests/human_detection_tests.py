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

class HumanDetectionTests(unittest.TestCase):

    def setUp(self):
        rospack = rospkg.RosPack()
        self.pkgDir = rospack.get_path('rapp_testing_tools')

        self.ch = RappPlatformAPI()

    def test_humanDetectionTest(self):
        image = path.join(self.pkgDir, 'test_data',
            'human_detection_samples', 'NAO_picture_3.png')

        valid_humans = [{
            'up_left_point': {'y': 30.0, 'x': 48.0},
            'down_right_point': {'y': 399.0, 'x': 232.0}
        }]

        response = self.ch.humanDetection(image)

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['humans'], valid_humans)

    def test_humanDetectionTest_wrongPath(self):
        image = path.join(self.pkgDir, '',
            'human_detection_samples', 'NAO_picture_3.png')
        response = self.ch.humanDetection(image)

        self.assertNotEqual(response['error'], u'')

    def test_humanDetectionTest_wrongPathType(self):
        image = 3
        response = self.ch.humanDetection(image)

        self.assertNotEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
