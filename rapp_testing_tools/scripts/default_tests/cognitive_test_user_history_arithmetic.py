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

__path__ = os.path.dirname(os.path.realpath(__file__))

from RappCloud import RappPlatformAPI

class RappInterfaceTest:

  def __init__(self):
    # self.msg = CognitiveGetHistory(
        # test_type='ArithmeticCts',
        # time_from=0,
        # time_to=10000000)
    self.ch = RappPlatformAPI()

  def execute(self):
    start_time = timeit.default_timer()
    # Call the RappCloud service
    response = self.ch.cognitiveGetHistory('ArithmeticCts', 0, 100000000)
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    error = response['error']
    records = response['records']

    if error != "":
      return [error, self.elapsed_time]

    # Check if the returned data are equal to the expected
    if len(records) > 0:
        return [True, self.elapsed_time]
    else:
        return ["Unexpected result : " + str(response), self.elapsed_time]

