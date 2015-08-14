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

import sys
import time
import os
from rapp_utilities import Utilities

class SetNoiseProfile:

  def __init__(self):
    self.utilities = Utilities()

  # Service callback for detecting silence
  def setNoise_profile(self, user, noise_audio_file, audio_file_type): 

    cleanup = []
    directory = "/tmp/rapp_platform_files/audio_processing/" + user
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

    # Making audio compatible to sphinx4
    if audio_file_type == 'nao_ogg':
      new_audio += ".wav"
      com_res = os.system("sox " + noise_audio_file + " " + new_audio)
      if com_res != 0:
        return "Error: Server sox malfunctioned"
      cleanup.append(new_audio)
    elif audio_file_type == "nao_wav_1_ch":
      pass
    elif audio_file_type == "nao_wav_4_ch":
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

