#!/usr/bin/env python

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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr


import os
import sys
import rospkg

class Sox:

    def profileNoise(self, input_file, output_profile, time_init, time_end):
        os.system("sox " + input_file + " -t null /dev/null trim " +\
                time_init + " " + time_end + " noiseprof " + output_profile)

    def denoise(self, input_file, output_file, noise_prof, perc):
        os.system("sox " + input_file + " " + output_file + " noisered " +\
        noise_prof + " " + perc)

    def changeRate(self, input_file, output_file,  new_rate):
        os.system("sox " + input_file + " -r " + new_rate + " " + output_file)

    def changeChannels(self, input_file, output_file, channels):
        os.system("sox " + input_file + " -c " + channels + " " + output_file)

    def oggToWav(self, input_file, output_file):
        os.system("sox " + input_file + " " + output_file)
