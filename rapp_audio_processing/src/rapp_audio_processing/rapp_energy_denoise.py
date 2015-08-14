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

