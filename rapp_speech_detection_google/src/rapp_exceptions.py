#!/usr/bin/python

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

## @class RappError
# Inherits Exception and is used to catch only RAPP-specific exceptions
class RappError(Exception):
  """Error handling in RAPP"""

  ## Default contructor
  def __init__(self, value):
    self.value = value

  ## Returns the error in a string form
  def __str__(self):
    return repr(self.value)
