#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# contact: klpanagi@gmail.com, etsardou@iti.gr


import sys
import os
import timeit
import argparse

__path__ = os.path.dirname(os.path.realpath(__file__))

## ------ Access the RappCloud python module ------- ##
from RappCloud import RappCloud

class RappInterfaceTest:

  def __init__(self):
    self.rappCloud = RappCloud()
    self.file_uri = __path__  + '/../test_data/speech_detection_samples/recording_sentence2.ogg'
    self.language = 'en'
    self.audio_source = 'nao_ogg'
    self.user = 'rapp'
    self.valid_words_found = ['check', 'my', 'mail']
    # self.valid_sentences = [
        # ['check', '-'],
        # ['check','my','mail'],
        # ['Tech', '-'],
        # ['take','-'],
        # ['10','-'],
        # ['check', 'minus']
    # ]

  def execute(self):

    start_time = timeit.default_timer()
    response = self.rappCloud.speech_detection_google(\
        self.file_uri,\
        self.audio_source,\
        self.user,\
        self.language)
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)

  def validate(self, response):
    error = response['error']
    if error != "":
      return [error, self.elapsed_time]

    words_found = response['words']
    alt_words_found = response['alternatives']
    if self.valid_words_found == words_found or self.valid_words_found in alt_words_found:
        return [True, self.elapsed_time]
    else:
        return ["Unexpected result : " + str(response), self.elapsed_time]

