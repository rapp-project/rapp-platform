#!/usr/bin/env python

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
import numpy as np
import matplotlib.pyplot as plt
import time
from os.path import expanduser

from rapp_platform_ros_communications.srv import (
  registerImageToOntologySrv,
  registerImageToOntologySrvResponse  
  )


class ImageRegistrationToOntology:
  
  def registerImage(self,req):
    res = registerImageToOntologySrvResponse()
    return res




