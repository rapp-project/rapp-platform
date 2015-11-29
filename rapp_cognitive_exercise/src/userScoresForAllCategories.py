#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr

import rospy
import sys
import calendar
import time
from datetime import datetime
from os.path import expanduser
from collections import OrderedDict



from rapp_platform_ros_communications.srv import (

  userScoreHistoryForAllCategoriesResponse

  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

## @class RecordUserCognitiveTestPerformance
# @brief Provides the necessary functions for selecting a cognitive exercise test
#
# It implements the cognitive exercise record user cognitive test performance service
class UserScoresForAllCategories:

  ## @brief The callback function of the cognitive exercise record user cognitive test performance service
  # @param req [rapp_platform_ros_communications::userScoresForAllCategoriesRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::userScoresForAllCategoriesRequest::Response&] The ROS service response
  # @exception Exception IndexError
  # @exception Exception AIOError
  def returnUserScores(self,req):
    try:
      res = userScoreHistoryForAllCategoriesResponse()      

    except IndexError:
      res.trace.append("Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format")
      res.success=False
    except IOError:
      print "Error: can\'t find login file or read data"
      res.success=False
      res.trace.append("Error: can\'t find login file or read data")
    return res





