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
from os.path import join

__path__ = os.path.dirname(os.path.realpath(__file__))

## ------ Access the RappCloud python module ------- ##
from RappCloud import RappCloud

class RappInterfaceTest:

  def __init__(self):
    self.rappCloud = RappCloud()
    rospack = rospkg.RosPack()
    pkgDir = rospack.get_path('rapp_testing_tools')

    self.request = {
        'png_file': join(pkgDir, 'test_data', 'path_planning', '523_m_obstacle_2.png'),
        'yaml_file': join(pkgDir, 'test_data', 'path_planning', '523_m_obstacle_2.yaml'),
        'user': 'rapp',
        'map_name': '523_m_obstacle_2'
    }

    self.validRes = {
        'success': True,
        'error': ''
    }


  def execute(self):
    start_time = timeit.default_timer()
    response = self.rappCloud.path_planning_upload_map(self.request['png_file'], \
            self.request['yaml_file'], self.request['user'], self.request['map_name'])
    print response
    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(response)

  def validate(self, response):
    error = response['error']
    if error != "":
      return [error, self.elapsed_time]

    if self.validRes == response:
      return [True, self.elapsed_time]
    else:
      return ["Unexpected result : " + str(response), self.elapsed_time]

