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

class EnergyDenoise:

  # Method for energy-based denoising
  def energyDenoise(self, audio_file, scale, denoised_audio_file, energy_denoising_debug):
    if not os.path.isfile(audio_file):
        return False
    samp_freq, signal = wavfile.read(audio_file)
    samples = signal.shape[0]
    sq_signal = signal * 1.0

    if energy_denoising_debug:
      timearray = arange(0, samples*1.0, 1)
      timearray /= samp_freq
      timearray *= 1000.0
      subplot(3,1,1)
      plot(timearray, signal, color = 'k')

    for i in range(0, len(sq_signal)):
      sq_signal[i] *= sq_signal[i]
    mean_sq = mean(sq_signal)

    for i in range(0, len(sq_signal)):
      if sq_signal[i] < scale * mean_sq:
        signal[i] = 0

    if energy_denoising_debug:
      timearray = arange(0, samples*1.0, 1)
      timearray /= samp_freq
      timearray *= 1000.0
      subplot(3,1,2)
      plot(timearray, signal, color = 'k')

    if energy_denoising_debug:
      show()

    wavfile.write(denoised_audio_file, samp_freq, signal)

    return True

