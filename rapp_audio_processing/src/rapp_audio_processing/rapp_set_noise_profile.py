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

import sys
import time
import os

from scipy.io import wavfile

from rapp_utilities import Utilities

## @class SetNoiseProfile
# Evaluates the noise profile for an audio file
class SetNoiseProfile:

  ## Performs initializations
  def __init__(self):
    self.utilities = Utilities()

  ## Evaluates the audio profile
  #
  # Handles service callback
  # rapp_audio_processing.AudioProcessing#setNoiseProfileCallback
  #
  # @param user             [string] The system user, for environmental variable access
  # @param noise_audio_file [string] Noise audio file path
  # @param audio_type       [string] Noise audio file's type
  def setNoise_profile(self, user, noise_audio_file, audio_file_type):

    cleanup = []
    directory = os.path.expanduser("~/rapp_platform_files/audio_processing/") + user
    if not os.path.isdir(directory):
      os.makedirs(directory)
      com_res = os.system("chmod 777 " + directory)
      if com_res != 0:
        return "Error: Server chmod malfunctioned"

    directory += "/noise_profile/"
    if not os.path.isdir(directory):
      os.makedirs(directory)
      com_res = os.system("chmod 777 " + directory)
      if com_res != 0:
        return "Error: Server chmod malfunctioned"

    noise_profile_file = directory
    new_audio = noise_audio_file

    if not os.path.isfile(new_audio):
      return "Error: The audio file does not exist"

    # Making audio compatible to sphinx4
    if audio_file_type == 'nao_ogg':
      if ".ogg" not in new_audio:
        return "Error: ogg type selected but file is of another type"
      new_audio += ".wav"
      com_res = os.system("sox " + noise_audio_file + " " + new_audio)
      if com_res != 0:
        return "Error: Server sox malfunctioned"
      cleanup.append(new_audio)

    elif audio_file_type == "nao_wav_1_ch":
      if ".wav" not in new_audio:
        return "Error: wav type 1 channel selected but file is of another type"
      samp_freq, signal = wavfile.read(new_audio)
      if len(signal.shape) != 1:
        return "Error: wav 1 ch declared but the audio file has " +\
            str(signal.shape[1]) + ' channels'
      pass

    elif audio_file_type == "nao_wav_4_ch":
      if ".wav" not in new_audio:
        return "Error: wav type 4 channels selected but file is of another type"
      samp_freq, signal = wavfile.read(new_audio)
      if len(signal.shape) != 2 or signal.shape[1] != 4:
        return "Error: wav 4 ch declared but the audio file has not 4 channels"
      new_audio += "_1ch.wav"
      com_res = os.system("sox " + noise_audio_file + " -c 1 -r 16000 " + \
          new_audio)
      if com_res != 0:
        return "Error: Server sox malfunctioned"
      cleanup.append(new_audio)
    else:
      success = ''
      success = "Non valid noise audio type"
      status = self.utilities.cleanup(cleanup)
      if status != True:
        success += " " + status
      return  success

    noise_profile_uri = directory + "/noise_profile_" + audio_file_type
    # Extract noise_profile
    com_res = os.system(\
            "sox " + new_audio + " -t null /dev/null trim 0.5 2.5 noiseprof "\
            + noise_profile_uri)
    if com_res != 0:
      return "Error: Server sox malfunctioned"

    com_res = os.system("chmod 777 " + noise_profile_uri)
    if com_res != 0:
      return "Error: Server chmod malfunctioned"

    status = self.utilities.cleanup(cleanup)
    if status != True:
      return status
    else:
      return "true"

