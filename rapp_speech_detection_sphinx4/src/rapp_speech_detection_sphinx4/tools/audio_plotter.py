#!/usr/bin/python

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


import sys
from pylab import *
from scipy.io import wavfile
from os import listdir
from os.path import isfile, join

folder = sys.argv[1]
files = [ f for f in listdir(folder) if isfile(join(folder, f)) ]
fnum = len(files)
counter = 0
for f in files:
    sampFreq, signal = wavfile.read(folder + f)
    samples = signal.shape[0]

    timearray = arange(0, samples*1.0, 1)
    timearray /= sampFreq
    timearray *= 1000.0
    subplot(fnum, 1, counter)
    plot(timearray, signal, color = 'k')
    xlabel(f)
    counter += 1

show()

