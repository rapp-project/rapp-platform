#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

from RappCloud import Service
from RappCloud.CloudMsgs import OntologyIsSubsuperclass


class RappInterfaceTest:

  def __init__(self):
    # Set the valid results
    self.valid_results = True;

    self.msg = OntologyIsSubsuperclass(
        parent_class='Oven',
        child_class='MicrowaveOven',
        recursive=True)
    self.svc = Service(msg=self.msg)


  def execute(self):
    start_time = timeit.default_timer()

    # Call the Python RappCloud service
    response = self.svc.call()
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    # Get the returned data
    error = response.error
    if error != "":
      return [error, self.elapsed_time]

    return_data = response.result
    # Check if the returned data are equal to the expected
    if self.valid_results == return_data:
      return [True, self.elapsed_time]
    else:
      return ["Unexpected result : " + str(return_data), self.elapsed_time]

