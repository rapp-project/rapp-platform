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

# Authors: Manos Tsardoulias
# contact: etsardou@iti.gr

import sys
import time
import os

class Utilities:

  # Service callback for detecting silence
  def cleanup(self, files):
    for f in files:
      if os.path.isfile(f) == False:
        return "Error: " + f + " is not a file"
      command = "rm " + f
      com_res = os.system(command)
      if com_res != 0:
        return "Error: Server rm malfunctioned"
    return True

