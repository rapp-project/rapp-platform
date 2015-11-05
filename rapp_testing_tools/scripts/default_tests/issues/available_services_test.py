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
    # Set the valid results
    self.valid_results = [
        u'ontology_subclasses_of',
        u'available_services',
        u'qr_detection',
        u'speech_detection_sphinx4',
        u'speech_detection_google',
        u'ontology_is_subsuperclass_of',
        u'face_detection',
        u'set_denoise_profile',
        u'ontology_superclasses_of',
        u'record_cognitive_test_performance',
        u'cognitive_test_chooser',
        u'text_to_speech'
    ]


  def execute(self):
    start_time = timeit.default_timer()
    # Call the Python RappCloud service
    response = self.rappCloud.available_services()
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    # Get the returned data
    error = response['error']
    if error != "":
      return [error, self.elapsed_time]

    return_data = response['services']
    # Check if the returned data are equal to the expected
    for ser in self.valid_results:
        if ser not in return_data:
            return ["Unexpected result : " + str(return_data), self.elapsed_time]
    return [True, self.elapsed_time]
