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

from RappCloud import Service
from RappCloud.CloudMsgs import (
    PathPlanningPlan2D,
    PathPlanningUploadMap
    )


class RappInterfaceTest:

  def __init__(self):
    rospack = rospkg.RosPack()
    pkgDir = rospack.get_path('rapp_testing_tools')
    testDatapath = path.join(pkgDir, 'test_data', 'path_planning')

    poseStart = {
        'header':{
            'seq': 0, 'stamp':{'sec': 0, 'nsecs': 0}, 'frame_id': ''
            },
        'pose': {
            'position': {'x': 10, 'y': 10, 'z': 20},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 0}
            }
        }

    poseGoal = {
        'header':{
            'seq': 0, 'stamp':{'sec': 0, 'nsecs': 0}, 'frame_id': ''
            },
        'pose': {
            'position': {'x': 120, 'y': 10, 'z': 20},
            'orientation': {'x': 0, 'y': 0, 'z': 20, 'w': 0}
            }
        }


    yamlFile = path.join(testDatapath, '523_m_obstacle_2.yaml')
    pngFile = path.join(testDatapath, '523_m_obstacle_2.png')

    self.ppUploadMapMsg = PathPlanningUploadMap(
        map_name='523_m_obstacle_2',
        yaml_file=yamlFile,
        png_file=pngFile
        )

    self.ppPlanMsg = PathPlanningPlan2D(
        map_name='523_m_obstacle_2',
        robot_type='NAO',
        algorithm='dijkstra',
        pose_start=poseStart,
        pose_goal=poseGoal
        )

    self.svc = Service()


  def execute(self):
    start_time = timeit.default_timer()
    resp = self.svc.call(self.ppUploadMapMsg)
    # If error occured while uploading map return error
    if resp.error != '':
       return self.validate(resp)

    resp = self.svc.call(self.ppPlanMsg)

    end_time = timeit.default_timer()
    self.elapsed_time = end_time - start_time
    return self.validate(resp)


  def validate(self, response):
    error = response.error
    if error != "":
      return [error, self.elapsed_time]
    else:
      return [True, self.elapsed_time]
