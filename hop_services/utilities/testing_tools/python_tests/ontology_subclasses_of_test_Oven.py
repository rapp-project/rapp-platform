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
module_path = __path__ + '/../../python'
sys.path.append(module_path)
from RappCloud import *

class RappInterfaceTest:

  def __init__(self):
    self.rappCloud = RappCloud()
    self.ontology_class = "Oven"
    # Set the valid results
    self.valid_results = [ 'http://knowrob.org/kb/knowrob.owl#MicrowaveOven', \
      'http://knowrob.org/kb/knowrob.owl#RegularOven', \
      'http://knowrob.org/kb/knowrob.owl#ToasterOven']

  def execute(self):

    start_time = timeit.default_timer()
    # Call the Python RappCloud service
    response = self.rappCloud.ontology_subclasses_of(self.ontology_class)
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)

  def validate(self, response):
    # Get the returned data
    return_data = response['results']
    error = response['error']
    if error != "":
      return [error, self.elapsed_time]
    # Check if the returned data are equal to the expected
    if self.valid_results == return_data:
      return [True, self.elapsed_time]
    else:
      return ["Unexpected result : " + str(return_data), self.elapsed_time]

