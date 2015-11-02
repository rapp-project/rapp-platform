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

import os
import timeit

__path__ = os.path.dirname(os.path.realpath(__file__))

## ------ Access the RappCloud python module ------- ##
from RappCloud import RappCloud

class RappInterfaceTest:

  def __init__(self):
    self.rappCloud = RappCloud()
    self.language = 'el'
    self.text = u'Καλησπέρα. Είμαι ο ναο.'
    self.destFilePath = '/tmp/ttsClient.wav'
    # Set the valid results

  def execute(self):
    start_time = timeit.default_timer()
    # Call the Python RappCloud service
    response = self.rappCloud.text_to_speech(self.text, self.language,
                                             self.destFilePath)
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)

  def validate(self, response):
    if response['error']:
        return [response['error'], self.elapsed_time]

    # Check if the returned data are equal to the expected
    if os.path.getsize(self.destFilePath) > 0:
        return [True, self.elapsed_time]
    else:
        return ["Unexpected result : " + 'Invalid size of audio data', self.elapsed_time]

