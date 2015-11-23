#!/usr/bin/env python

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
