#! /usr/bin/env python

import sys
import unittest
import roslib

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import rapp_audio_processing

from rapp_platform_ros_communications.srv import (  
  AudioProcessingDenoiseSrv, 
  AudioProcessingDenoiseSrvResponse,
  
  AudioProcessingSetNoiseProfileSrv,
  AudioProcessingSetNoiseProfileSrvResponse,

  AudioProcessingDetectSilenceSrv,
  AudioProcessingDetectSilenceSrvResponse
  )

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        self.audio = rapp_audio_processing.AudioProcessing

    def tearDown(self):
        self.audio = None

    def test_simpleTest(self):
        self.assertEqual(0, 0)

