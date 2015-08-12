#! /usr/bin/env python

import sys
import unittest
import roslib
import rospy

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import AudioProcessing


class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        #self.audio = AudioProcessing()
        pass

    def tearDown(self):
        # self.audio = None
        pass 
    def test_cleanupExistentFile(self):
        # Create an empty file
        name = '/tmp/unit_test_file'
        file = open(name, 'w')
        file.close()

        to_be_erased = []
        to_be_erased.append(name)
        # output = self.audio.cleanup(to_be_erased)
        # self.assertEqual(True, output)

