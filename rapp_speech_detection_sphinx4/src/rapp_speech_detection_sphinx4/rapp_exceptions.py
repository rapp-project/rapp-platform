#!/usr/bin/python

import sys

class RappException(Exception):
  def __init__(self, value):
    self.value = value
  def __str__(self):
    return repr(self.value)
