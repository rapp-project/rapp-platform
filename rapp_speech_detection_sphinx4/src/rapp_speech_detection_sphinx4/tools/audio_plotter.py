#!/usr/bin/python

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

