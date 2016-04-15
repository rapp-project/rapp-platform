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

## ------ Access the RappCloud python module ------- ##
from RappCloud import CognitiveTestSelect

class RappInterfaceTest:

  def __init__(self):
    self.svc = CognitiveTestSelect(test_type='ReasoningCts')


  def execute(self):
    start_time = timeit.default_timer()
    # Call the Python RappCloud service
    response = self.svc.call()
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    error = response.error
    if error != "":
      return [error, self.elapsed_time]

    # Get the returned data
    test_instance = response.test_instance
    test_type = response.test_type
    test_sub_type = response.test_subtype
    questions = response.questions
    possib_ans = response.possib_ans
    correct_ans = response.correct_ans
    # Check if the returned data are equal to the expected
    if test_instance and test_type and test_sub_type and questions \
            and possib_ans and correct_ans:
        return [True, self.elapsed_time]
    else:
        return ["Unexpected result : " + str(response), self.elapsed_time]

