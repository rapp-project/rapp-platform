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

from RappCloud import RappPlatformService
## from RappCloud.CloudMsgs import FaceDetection

class RappInterfaceTest:

  def __init__(self):
    ## Initialize a new RappPlatformService
    self.svc = RappPlatformService(self.msg)
    ## Create a CloudMsg
    # self.msg = FaceDetection()
    ## Set CloudMsg properties
    # self.file_uri = __path__ + '/../test_data/YOUR_DATA_HERE_IF_NEEDED'
    ## Set the valid results
    # self.valid_results = {}

  def execute(self):
    start_time = timeit.default_timer()
    ## Call the Python RappCloud service
    self.svc.call(self.msg)
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    ## Check if an error occured.
    error = response.error
    if error != "":
      return [error, self.elapsed_time]
    ## Validate response
    if self.valid_results == response.serialize():
      return [True, self.elapsed_time]
    else:
      return ["Unexpected result : " + str(return_data), self.elapsed_time]

