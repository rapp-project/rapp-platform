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

from os import path
import os
import timeit
import unittest
import rospkg

__path__ = os.path.dirname(os.path.realpath(__file__))

from RappCloud import RappPlatformAPI

class FaceDetectionTests(unittest.TestCase):

    def setUp(self):
        self.ch = RappPlatformAPI()    
        
        rospack = rospkg.RosPack()
        self.pkgDir = rospack.get_path('rapp_testing_tools')
       
    def test_closeStraight(self):
        imagepath = path.join(self.pkgDir, 'test_data',
            'face_samples', 'klpanagi_close_straight.jpg')
        response = self.ch.faceDetection(imagepath)

        valid_faces = [{
            'up_left_point': {'y': 450.0, 'x': 640.0},
            'down_right_point': {'y': 662.0, 'x': 852.0}
        }]

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['faces'], valid_faces)

    def test_lenna(self):
        imagepath = path.join(self.pkgDir, 'test_data', 'Lenna.png')
        response = self.ch.faceDetection(imagepath)
        
        valid_faces = [{
            'up_left_point': {'y': 201.0, 'x': 213.0},
            'down_right_point': {'y': 378.0, 'x': 390.0}
        }]

        self.assertEqual(response['error'], u'')
        self.assertEqual(response['faces'], valid_faces)

    def test_multipleFaces(self):
        imagepath = path.join(self.pkgDir, 'test_data', 'face_samples', \
                'multi_faces_frames', 'multi_faces.jpg')
        response = self.ch.faceDetection(imagepath)
       
        self.assertEqual(response['error'], u'')
        self.assertEqual(len(response['faces']), 3)

    def test_notExistentFile(self):
        imagepath = path.join(self.pkgDir, 'face_samples', \
                'multi_faces_frames', 'multi_faces.jpg')
        response = self.ch.faceDetection(imagepath)
       
        self.assertNotEqual(response['error'], u'')

    def test_erroneousFileType(self):
        imagepath = 3
        response = self.ch.faceDetection(imagepath)
       
        self.assertNotEqual(response['error'], u'')

if __name__ == "__main__":
    unittest.main()
