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

class PathPlanningTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()

        rospack = rospkg.RosPack()
        self.pkgDir = rospack.get_path('rapp_testing_tools')

    def test_plan_2d(self):
        testDatapath = path.join(self.pkgDir, 'test_data', 'path_planning')

        poseStart = {
            'header':{
             'seq': 0, 'stamp':{'sec': 0, 'nsecs': 0}, 'frame_id': ''
            },
            'pose': {
             'position': {'x': 1, 'y': 1, 'z': 0},
             'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 0}
            }
        }

        poseGoal = {
            'header':{
                'seq': 0, 'stamp':{'sec': 0, 'nsecs': 0}, 'frame_id': ''
            },
            'pose': {
                'position': {'x': 5.3, 'y': 4, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 20, 'w': 0}
            }
        }

        yamlFile = path.join(testDatapath, '523_m_obstacle_2.yaml')
        pngFile = path.join(testDatapath, '523_m_obstacle_2.png')
        map_name='523_m_obstacle_2'

        resp = self.ch.pathPlanningUploadMap(map_name, pngFile, yamlFile)
        self.assertEqual(resp['error'], u'')

        resp = self.ch.pathPlanningPlan2D(map_name, 'NAO', poseStart,\
            poseGoal)
        self.assertEqual(resp['error'], u'')      
        self.assertEqual(resp['plan_found'], 0)      

    def test_upload_map(self):
        testDatapath = path.join(self.pkgDir, 'test_data', 'path_planning')
        yamlFile = path.join(testDatapath, '523_m_obstacle_2.yaml')
        pngFile = path.join(testDatapath, '523_m_obstacle_2.png')
        map_name='523_m_obstacle_2'

        resp = self.ch.pathPlanningUploadMap(map_name, pngFile, yamlFile)
        self.assertEqual(resp['error'], u'')

if __name__ == "__main__":
    unittest.main()
