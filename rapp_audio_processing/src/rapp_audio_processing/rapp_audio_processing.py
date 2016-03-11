#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

## @class AudioProcessing
# Provides audio processing utilities
#
# Implements service servers to perform denoising, audio transformation etc.
class AudioProcessing:

  ## Constructor performing initializations
  def __init__(self):

    ## Instantiates rapp_detect_silence.DetectSilence
    self._detect_silence_module = DetectSilence()
    ## Instantiates rapp_energy_denoise.EnergyDenoise
    self._energy_denoise_module = EnergyDenoise()
    ## Instantiates rapp_sox_denoise.SoxDenoise
    self._sox_denoise_module = SoxDenoise()
    ## Instantiates rapp_utilities.Utilities
    self._utilities_module = Utilities()
    ## Instantiates rapp_set_noise_profile.SetNoiseProfile
    self._set_noise_profile_module = SetNoiseProfile()
    ## Instantiates rapp_transform_audio.TransformAudio
    self._transform_audio_module= TransformAudio()

    # Parameters acquisition
    set_noise_profile_topic = \
        rospy.get_param("rapp_audio_processing_set_noise_profile_topic")
    denoise_topic = \
        rospy.get_param("rapp_audio_processing_denoise_topic")
    energy_denoise_topic = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")
    detect_silence_topic = \
        rospy.get_param("rapp_audio_processing_detect_silence_topic")
    transform_audio_topic = \
        rospy.get_param("rapp_audio_processing_transform_audio_topic")

    if(not set_noise_profile_topic):
      rospy.logerror("Audio processing noise profiling topic param not found")
    if(not denoise_topic):
      rospy.logerror("Audio processing denoise topic param not found")
    if(not energy_denoise_topic):
      rospy.logerror("Audio processing energy denoise topic param not found")
    if(not detect_silence_topic):
      rospy.logerror("Audio processing detect silence topic param not found")
    if(not transform_audio_topic):
      rospy.logerror("Audio processing noise transform audio topic param not found")

    # Check for denoising debug mode. DO NOT make this true when in production
    ## Energy denoising degug flag
    self._energy_denoising_debug = False
    self._energy_denoising_debug = \
        rospy.get_param("rapp_audio_processing_energy_denoising_debug")
    if not self._energy_denoising_debug:
      self._energy_denoising_debug = False
    else:
      self._energy_denoising_debug = True

    # Create set noise profile services
    set_noise_profile_service = rospy.Service(set_noise_profile_topic, \
        AudioProcessingSetNoiseProfileSrv, self.setNoiseProfileCallback)
      # Create sox denoise services
    denoise_service = rospy.Service( \
        denoise_topic, AudioProcessingDenoiseSrv, \
        self.denoiseCallback)
    # Create energy denoise services
    energy_denoise_service = rospy.Service( \
        energy_denoise_topic, AudioProcessingDenoiseSrv, \
        self.energyDenoiseCallback)
    # Create detect silence services
    detect_silence_service = rospy.Service( \
        detect_silence_topic, AudioProcessingDetectSilenceSrv, \
        self.detectSilenceCallback)
    # Create transform audio services
    transform_audio = rospy.Service( transform_audio_topic, \
        AudioProcessingTransformAudioSrv, self.transformAudioCallback)


  ## Service callback for setting noise profile
  #
  # @param req [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingSetNoiseProfileSrv] The set noise profile request
  #
  # @return res [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingSetNoiseProfileSrvResponse] The set noise profile response
  def setNoiseProfileCallback(self, req):
    res = AudioProcessingSetNoiseProfileSrvResponse()

    #-------------------------set noise profile-------------------------#
    ret = self._set_noise_profile_module.setNoise_profile(\
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

  ## Service callback for denoising
  #
  # @param req [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingDenoiseSrv] The denoise request
  #
  # @return res [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingDenoiseSrvResponse] The denoise response
  def denoiseCallback(self, req):
    res = AudioProcessingDenoiseSrvResponse()
    res.success = self._sox_denoise_module.soxDenoise(\
            req.user,\
            req.audio_type,\
            req.audio_file,\
            req.denoised_audio_file,\
            req.scale)
    return res

  ## Service callback for Detecting silence
  #
  # @param req [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingDetectSilence] The detect silence request
  #
  # @return res [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingDetectSilenceSrvResponse] The detect silence response
  def detectSilenceCallback(self, req):
    res = AudioProcessingDetectSilenceSrvResponse()
    [res.level, res.silence] = self._detect_silence_module.detectSilence(\
            req.audio_file, req.threshold)
    if res.silence == True:
        res.silence = "true"
    else:
        res.silence = "false"
    return res

  ## Service callback for Energy denoising
  #
  # @param req [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingDenoiseSrv] The denoise request
  #
  # @return res [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingDenoiseSrvResponse] The energy denoise response
  def energyDenoiseCallback(self, req):
    res = AudioProcessingDenoiseSrvResponse()
    output = self._energy_denoise_module.energyDenoise(\
          req.audio_file, req.scale, req.denoised_audio_file,\
          self._energy_denoising_debug)
    if output == True:
        res.success = "true"
    else:
        res.success = "false"
    return res

  ## Service callback for Audio transformation
  #
  # @param req [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingTransformAudioSrv] The transform audio request
  #
  # @return res [rapp_platform_ros_comminications::AudioProcessing::AudioProcessingTransformAudioSrvResponse] The transform audio response
  def transformAudioCallback(self, req):
      res = AudioProcessingTransformAudioSrvResponse()

      [ res.error, res.fullpath ] = \
          self._transform_audio_module.transform_audio( \
              req.source_type, req.source_name, req.target_type, \
              req.target_name, req.target_channels, req.target_rate )

      return res

# Main function
if __name__ == "__main__":
  rospy.init_node('AudioProcessing')
  AudioProcessingNode = AudioProcessing()
  rospy.spin()
