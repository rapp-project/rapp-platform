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
from pylab import *
from scipy.io import wavfile

class SoxDenoise :

  # Service callback for detecting silence
  def soxDenoise(self, user, audio_type, audio_file, denoised_audio_file, scale):
    if not os.path.isfile(audio_file):
        return "The audio file does not exist"

    if scale < 0 or scale > 1:
        return "Invalid scale. Scale must be between [0,1]"

    if ".wav" not in audio_file:
        return "The file for denoising is not wav"
    samp_freq, signal = wavfile.read(audio_file)
    if len(signal.shape) != 1:
        return "The file for denoising has not 1 channel"


    directory = os.path.expanduser("~/rapp_platform_files/audio_processing/") + user
    noise_profile = directory + "/noise_profile/noise_profile_" + audio_type

    if not os.path.isfile(noise_profile):
        return "No noise profile for the " + audio_type + " type exists"

    command = "sox " + audio_file + " " + denoised_audio_file +\
            " noisered " + noise_profile + " " + str(scale)
    com_res = os.system(command)

    if com_res != 0:
        return "System sox malfunctioned"
    else:
        return "true"

