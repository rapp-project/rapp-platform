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

## @class DetectSilence
# Performs silence detection on an audio file
class DetectSilence:

  ## Detects silence
  #
  # Handles service callback
  # rapp_audio_processing.AudioProcessing#detectSilenceCallback
  #
  # @param audio_file [string] Audio file path
  # @param threshold  [float] Silence threshold
  #
  # @return rsd_sq      [float] Noise relative standard deviation
  # @return has_silence [bool] Indicates the existence of silence
  def detectSilence(self, audio_file, threshold):
    if not os.path.isfile(audio_file):
        return [-1, False]
    samp_freq, signal = wavfile.read(audio_file)
    sq_signal = signal * 1.0
    for i in range(0, len(sq_signal)):
      sq_signal[i] *= sq_signal[i]
    mean_sq = mean(sq_signal)
    std_sq = std(sq_signal)
    rsd_sq = std_sq / mean_sq
    has_silence = False
    if rsd_sq > threshold:
        has_silence = False
    else:
        has_silence = True
    return [rsd_sq, has_silence]

