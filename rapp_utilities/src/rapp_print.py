#!/usr/bin/python
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

import sys
import os
import inspect
import rospy

def rapp_print(var, verbosity='DEBUG'):

  callerframerecord = inspect.stack()[1]

  frame = callerframerecord[0]
  info = inspect.getframeinfo(frame)

  if verbosity == 'DEBUG':
    rospy.logdebug('[ FILE: ' + os.path.basename(info.filename) + \
        ', FUNCTION: ' + info.function + ', LINE: ' + str(info.lineno) + \
        ' ]: ' + str(var) )
  elif verbosity == 'INFO':
    rospy.loginfo('[ FILE: ' + os.path.basename(info.filename) + \
        ', FUNCTION: ' + info.function + ', LINE: ' + str(info.lineno) + \
        ' ]: ' + str(var) )
  elif verbosity == 'WARN':
    rospy.logwarn('[ FILE: ' + os.path.basename(info.filename) + \
        ', FUNCTION: ' + info.function + ', LINE: ' + str(info.lineno) + \
        ' ]: ' + str(var) )
  elif verbosity == 'ERROR':
    rospy.logerr('[ FILE: ' + os.path.basename(info.filename) + \
        ', FUNCTION: ' + info.function + ', LINE: ' + str(info.lineno) + \
        ' ]: ' + str(var) )
  elif verbosity == 'FATAL':
    rospy.logfatal('[ FILE: ' + os.path.basename(info.filename) + \
        ', FUNCTION: ' + info.function + ', LINE: ' + str(info.lineno) + \
        ' ]: ' + str(var) )
