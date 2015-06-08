#!/usr/bin/env python
# -*- coding: utf-8 -*-

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# contact: klpanagi@gmail.com, etsardou@iti.gr

import sys
import os
import timeit
import argparse
from os import listdir
from os.path import isfile, join
import importlib

__path__ = os.path.dirname(os.path.realpath(__file__))

## ------ Access the RappCloud python module ------- ##
module_path = __path__ + '/../python'
sys.path.append(module_path)
## ------------------------------------------------##

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def main(): 
  folder = __path__ + "/python_tests"
  sys.path.append(folder)
  if len(sys.argv) != 1:
    files = sys.argv[1:]
  else:
    files = [ f for f in listdir(folder) if isfile(join(folder, f)) ]
  for f in files:
    clean_file = f.split(".")
    if clean_file[1] != "py" or clean_file[0] == "template":
      continue
    module = importlib.import_module(clean_file[0])
    tmp = module.RappInterfaceTest()
    [error_code, time] = tmp.execute()
    print module.__name__,
    if error_code != True:
      print bcolors.FAIL + "FAIL [" + error_code + "]" + bcolors.ENDC + " " + str(time)
    else:
      print bcolors.OKGREEN + "SUCCESS" + bcolors.ENDC + " " + str(time)

if __name__ == "__main__":
  main()

