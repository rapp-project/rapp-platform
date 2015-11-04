#!/usr/bin/env python
# -*- encode: utf-8 -*-

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

# Authors: Manos Tsardoulias
# contact: etsardou@iti.gr

import rospy
import sys
import time
import os
from pylab import *
from scipy.io import wavfile

from rapp_platform_ros_communications.srv import (
  AudioProcessingDenoiseSrv,
  AudioProcessingDenoiseSrvResponse,

  AudioProcessingSetNoiseProfileSrv,
  AudioProcessingSetNoiseProfileSrvResponse,

  AudioProcessingDetectSilenceSrv,
  AudioProcessingDetectSilenceSrvResponse,

  AudioProcessingTransformAudioSrv,
  AudioProcessingTransformAudioSrvResponse
  )

from rapp_platform_ros_communications.srv import (
  fetchDataSrv,
  fetchDataSrvRequest
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from std_msgs.msg import (
  String
  )

from rapp_detect_silence import DetectSilence
from rapp_energy_denoise import EnergyDenoise
from rapp_sox_denoise import SoxDenoise
from rapp_utilities import Utilities
from rapp_set_noise_profile import SetNoiseProfile
from rapp_transform_audio import TransformAudio

class AudioProcessing:

  # Constructor performing initializations
  def __init__(self):

    self.detect_silence_module = DetectSilence()
    self.energy_denoise_module = EnergyDenoise()
    self.sox_denoise_module = SoxDenoise()
    self.utilities_module = Utilities()
    self.set_noise_profile_module = SetNoiseProfile()
    self.transform_audio_module= TransformAudio()

    # Parameters acquisition
    self.set_noise_profile_topic = \
        rospy.get_param("rapp_audio_processing_set_noise_profile_topic")
    self.denoise_topic = \
        rospy.get_param("rapp_audio_processing_denoise_topic")
    self.energy_denoise_topic = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")
    self.detect_silence_topic = \
        rospy.get_param("rapp_audio_processing_detect_silence_topic")
    self.transform_audio_topic = \
        rospy.get_param("rapp_audio_processing_transform_audio_topic")
    self.threads = \
        rospy.get_param("rapp_audio_processing_threads")

    if(not self.set_noise_profile_topic):
      rospy.logerror("Audio processing noise profiling topic param not found")
    if(not self.denoise_topic):
      rospy.logerror("Audio processing denoise topic param not found")
    if(not self.energy_denoise_topic):
      rospy.logerror("Audio processing energy denoise topic param not found")
    if(not self.detect_silence_topic):
      rospy.logerror("Audio processing detect silence topic param not found")
    if(not self.transform_audio_topic):
      rospy.logerror("Audio processing noise transform audio topic param not found")
    if not self.threads:
      rospy.logerror("Audio processing threads param not found")

    # Check for denoising debug mode. DO NOT make this true when in production
    self.energy_denoising_debug = False
    self.energy_denoising_debug = \
        rospy.get_param("rapp_audio_processing_energy_denoising_debug")
    if not self.energy_denoising_debug:
      self.energy_denoising_debug = False
    else:
      self.energy_denoising_debug = True

    # Create set noise profile services
    self.set_noise_profile_service = rospy.Service(self.set_noise_profile_topic, \
        AudioProcessingSetNoiseProfileSrv, self.setNoiseProfile)
    for i in range(0, self.threads):
        rospy.Service(self.set_noise_profile_topic + '_' + str(i),\
        AudioProcessingSetNoiseProfileSrv, self.setNoiseProfile)
    # Create sox denoise services
    self.denoise_service = rospy.Service( \
        self.denoise_topic, AudioProcessingDenoiseSrv, \
        self.denoise)
    for i in range(0, self.threads):
        rospy.Service(self.denoise_topic + '_' + str(i),\
        AudioProcessingDenoiseSrv, self.denoise)
    # Create energy denoise services
    self.energy_denoise_service = rospy.Service( \
        self.energy_denoise_topic, AudioProcessingDenoiseSrv, \
        self.energy_denoise)
    for i in range(0, self.threads):
        rospy.Service(self.energy_denoise_topic + '_' + str(i),\
        AudioProcessingDenoiseSrv,
        self.energy_denoise)
    # Create detect silence services
    self.detect_silence_service = rospy.Service( \
        self.detect_silence_topic, AudioProcessingDetectSilenceSrv, \
        self.detect_silence)
    for i in range(0, self.threads):
        rospy.Service(self.detect_silence_topic + '_' + str(i),\
        AudioProcessingDetectSilenceSrv,
        self.detect_silence)
    # Create transform audio services
    self.transform_audio = rospy.Service( self.transform_audio_topic, \
        AudioProcessingTransformAudioSrv, self.transform_audio)
    for i in range(0, self.threads):
        rospy.Service( self.transform_audio_topic + '_' + str(i), \
            AudioProcessingTransformAudioSrv, self.transform_audio)

    self.serv_db_topic = rospy.get_param("rapp_mysql_wrapper_user_fetch_data_topic")

# Service callback for setting noise profile
  def setNoiseProfile(self, req):
    res = AudioProcessingSetNoiseProfileSrvResponse()

    #-------------------------Check with database-------------------------#
    authentication_service = rospy.ServiceProxy(self.serv_db_topic, fetchDataSrv)
    req_db = fetchDataSrv()
    req_db.req_cols=["username"]
    entry1=["username", req.user]
    req_db.where_data=[StringArrayMsg(s=entry1)]

    resp = authentication_service(req_db.req_cols, req_db.where_data)
    if resp.success.data != True or len(resp.res_data) == 0:
      res.success = "false"
      res.error = "Non authenticated user"
      return res

    #-------------------------set noise profile-------------------------#
    ret = self.set_noise_profile_module.setNoise_profile(\
            req.user,\
            req.noise_audio_file,\
            req.audio_file_type)
    if ret == 'true':
        res.success = ret
        res.error = ''
    else:
        res.success = 'false'
        res.error = ret
    return res

  # Service callback for handling denoising
  def denoise(self, req):
    res = AudioProcessingDenoiseSrvResponse()
    res.success = self.sox_denoise_module.soxDenoise(\
            req.user,\
            req.audio_type,\
            req.audio_file,\
            req.denoised_audio_file,\
            req.scale)
    return res

  # Service callback for detecting silence
  def detect_silence(self, req):
    res = AudioProcessingDetectSilenceSrvResponse()
    [res.level, res.silence] = self.detect_silence_module.detectSilence(\
            req.audio_file, req.threshold)
    if res.silence == True:
        res.silence = "true"
    else:
        res.silence = "false"
    return res

  # Service callback for energy denoising
  def energy_denoise(self, req):
    res = AudioProcessingDenoiseSrvResponse()
    output = self.energy_denoise_module.energyDenoise(\
          req.audio_file, req.scale, req.denoised_audio_file,\
          self.energy_denoising_debug)
    if output == True:
        res.success = "true"
    else:
        res.success = "false"
    return res

  # Service callback for audio transformation
  def transform_audio(self, req):
      res = AudioProcessingTransformAudioSrvResponse()

      [ res.error, res.fullpath ] = \
          self.transform_audio_module.transformAudio( \
              req.source_type, req.source_name, req.target_type, \
              req.target_name, req.target_channels, req.target_rate )

      return res


  # Cleanup method
  def cleanup(self, clean):
    return self.utilities_module.cleanup(clean)

# Main function
if __name__ == "__main__":
  rospy.init_node('AudioProcessing')
  AudioProcessingNode = AudioProcessing()
  rospy.spin()
