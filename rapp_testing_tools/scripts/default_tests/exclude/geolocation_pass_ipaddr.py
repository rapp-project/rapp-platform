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

from RappCloud import Service
from RappCloud.CloudMsgs import Geolocation


class RappInterfaceTest:

  def __init__(self):
    self.validResults = {
        'city': 'San Francisco',
        'zip': '94107',
        'country': 'United States',
        'region': 'California',
        'country_code': 'US',
        'longtitude': -122.39330291748047,
        'latitude': 37.76969909667969,
        'timezone': 'America/Los_Angeles',
        'error': ''
    }

    self.msg = Geolocation(ipaddr='104.16.115.182')
    self.svc = Service(self.msg)


  def execute(self):
    start_time = timeit.default_timer()
    # Call the Python RappCloud service
    response = self.svc.call()

    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)


  def validate(self, response):
    error = response.error
    print response.serialize()

    if self.validResults == response.serialize():
        return [True, self.elapsed_time]
    else:
        return [error, self.elapsed_time]

