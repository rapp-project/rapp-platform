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
from threading import Thread, Lock

__path__ = os.path.dirname(os.path.realpath(__file__))

## ------ Access the RappCloud python module ------- ##
module_path = __path__ + '/../python'
sys.path.append(module_path)
## ------------------------------------------------##

# Mutex lock used 
mutex = Lock()
threaded = False

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



def execute(module):
  tmp = module.RappInterfaceTest()
  [error_code, time] = tmp.execute()
  print module.__name__,

  if threaded:
    mutex.acquire(True)
  if error_code != True:
    print bcolors.FAIL + "FAIL [" + error_code + "]" + bcolors.ENDC + " " + str(time)
  else:
    print bcolors.OKGREEN + "SUCCESS" + bcolors.ENDC + " " + str(time)
  if threaded:
    mutex.release()



def main(): 
  folder = __path__ + "/python_tests"
  sys.path.append(folder)

  # ---------------------------------------------- #
  parser = argparse.ArgumentParser(description= \
    'RAPP Platform front-end hop-service invocation tests')

  parser.add_argument('-i','--name', help='Test File Name to execute. \033[1;34mall==all\033[0m',\
    dest='fileName', action='store', nargs='+', type=str)

  parser.add_argument('-n', '--num-calls', dest='numCalls', action='store', \
    help='Number of times to run the test', type=int, default=1)

  parser.add_argument('-t', '--threaded', dest='threaded', action='store_true', \
    help='Enable threaded mode')
  args =  parser.parse_args( ) # Parse console arguments
  # --------------------------------------------- #

  ## ------------------- Parse arguments ---------------------------- ##
  if args.fileName == None:
    # Find and run all the tests located under python_tests dirrectory
    files = [ f for f in listdir(folder) if isfile(join(folder, f)) ]
  else:
    files = args.fileName # Input test files from arguments

  numCalls = args.numCalls
  # If threaded mode is enabled
  if args.threaded:
    threaded = True
    threads = []
  else:
    threaded = False

  ## ---------------------------------------------------------------- ##

  
  # -- Loop through test files to be executed -- #
  for f in files:
    clean_file = f.split(".")
    if clean_file[1] != "py" or clean_file[0] == "template":
      continue
    module = importlib.import_module(clean_file[0])

    for i in range(0, numCalls):
      if threaded: 
        thread = Thread(target=execute, args=(module, ))
        thread.start()
        threads.append(thread)
      else:
        execute(module)
  # ------------------------------------------- #

  if threaded:
    # Wait for all threads to complete
    for t in threads:
      t.join()
    
if __name__ == "__main__":
  main()

