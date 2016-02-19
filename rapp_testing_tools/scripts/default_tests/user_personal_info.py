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
from RappCloud import RappCloud

class RappInterfaceTest:

  def __init__(self):
    self.rappCloud = RappCloud()
    self.user = "rapp"
    self.valid_results = {
        'user_info': {
            'username': 'rapp',
            'fistname': 'rapp',
            'lastname': 'rapp',
            'email': 'testmail.rapp.com',
            'language': 'el',
            'ontology_alias': 'Person_DpphmPqg',
            'usrgroup': '5',
            'created': '2015-11-02 10:47:32'
        },
        'error': ''
    }


  def execute(self):
    start_time = timeit.default_timer()
    # Call the Python RappCloud service
    response = self.rappCloud.user_personal_info(self.user)
    print response
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    error = response['error']
    if error != "":
      return [error, self.elapsed_time]

    # Check if the returned data are equal to the expected
    for i in self.valid_results:
        if i in response['user_info']:
            print 'boom'
        return [True, self.elapsed_time]
    else:
        return ["Unexpected result : " + str(response), self.elapsed_time]

