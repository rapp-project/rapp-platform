#!/usr/bin/python

import sys

class RappError(Exception):
  """Error handling in RAPP"""

  def __init__(self, value):
    self.value = value

  def __str__(self):
    return repr(self.value)
