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


import os
import timeit
import rospkg
from os import path

__path__ = os.path.dirname(os.path.realpath(__file__))

## ------ Access the RappCloud python module ------- ##
from RappCloud import QrDetection

class RappInterfaceTest:

  def __init__(self):
    rospack = rospkg.RosPack()
    pkgDir = rospack.get_path('rapp_testing_tools')
    imagepath = path.join(pkgDir, 'test_data','qr_samples', 'easyMediumQr.jpg')
    self.svc = QrDetection(image=imagepath)

    self.valid_results = {
        'qr_centers': [{'y': 498, 'x': 632}],
        'qr_messages': ['Hello there'],
        'error': ''
        }


  def execute(self):
    start_time = timeit.default_timer()
    response = self.svc.call()
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    error = response.error
    if error != "":
      return [error, self.elapsed_time]

    if self.valid_results == response.serialize():
      return [True, self.elapsed_time]
    else:
      return ["Unexpected result : " + str(return_data), self.elapsed_time]

