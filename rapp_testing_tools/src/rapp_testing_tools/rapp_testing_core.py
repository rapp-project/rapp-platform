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
import time
import argparse
from os import listdir
from os.path import isfile, join
import importlib
from threading import Thread, Lock
# import roslib
import rospkg
import rospy
import yaml


__path__ = os.path.dirname(os.path.realpath(__file__))

# Mutex lock used when threaded.
mutex = Lock()

## --------- Test Classess ---------- ##
testClasses = [
    'face-detection',
    'qr-detection',
    'speech-detection',
    'speech-detection-sphinx4',
    'speech-detection-google',
    'ontology',
    'cognitive',
    'tts'
]
## --------------------------------- ##

testClassMatch = {
    'face-detection' : 'face',
    'qr-detection' : 'qr',
    'speech-detection' : 'speech',
    'speech-detection-sphinx4' : 'sphinx4',
    'speech-detection-google' : 'google',
    'ontology' : 'ontology',
    'cognitive': 'cognitive',
    'tts': 'text_to_speech'
}

results = {
    'success' : [],
    'failed' : [],
    'num_tests': 0
}


## ------------- Console colors -------------- ##
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    YELLOW = '\033[93m'
## ------------------------------------------ ##



##
#   @brief Execute input test module
#
#   @param module Imported test module to execute.
#   @param id This test execution id
##
def execute(module, id):
    tmp = module.RappInterfaceTest()
    [error_code, time] = tmp.execute()

    mutex.acquire(True)
    results['num_tests'] += 1
    print "\n\033[1;35m**  Test [%s] -- %s  **\033[0m" % (module.__name__, id)
    print "Execution time: " + str(time) + "sec"
    if error_code != True:
        results['failed'].append(module.__name__)
        print bcolors.FAIL + "[FAIL]:"
        print error_code + bcolors.ENDC
    else:
        results['success'].append(module.__name__)
        print bcolors.OKGREEN + "SUCCESS" + bcolors.ENDC
    mutex.release()


##
#   @brief Parse input arguments.
##
def parse_args():
    parser = argparse.ArgumentParser(description= \
            'RAPP Platform front-end hop-service invocation tests')

    parser.add_argument('-i','--name',\
            help='Test file name to execute.',\
            dest='fileName', action='store', nargs='+', type=str)

    parser.add_argument('-n', '--num-calls', dest='numCalls', action='store', \
            help='Number of times to run the test', type=int, default=1)

    parser.add_argument('-t', '--threaded', dest='threaded',\
            action='store_true', help='Enable threaded mode')

    parser.add_argument('-c', '--class', dest='testClass', action='store', \
            help='Tests class. "face-detection", "speech-detection"...', type=str)

    args =  parser.parse_args( ) # Parse console arguments
    return args


##
#   @brief Parse and get all given tests path directories, plus the default
#   ones.
#
#   @return Array of tests paths.
##
def load_tests_paths():
    ## This is the default directory where tests are stored
    testPaths = [join(join(__pkgDir__, 'scripts'), 'default_tests')]
    cfgFile = join(__pkgDir__, 'config/params.yaml')

    try:
        with open(cfgFile, 'r') as ymlFile:
            cfg = yaml.safe_load(ymlFile)
    except Exception as e:
        print e
        sys.exit(1)

    if 'tests_path' in cfg:
        extPaths = cfg['tests_path']

    if extPaths is not None and len(extPaths) > 0:
        for p in extPaths:
            testPaths += [p] if os.path.isdir(p) else []
    else:
        pass
    return testPaths


##
#   @brief Append directory paths, given as input into the global system path.
#       This is usefull in order to load test files under those directories.
##
def append_to_system_path(paths):
    for p in paths:
        sys.path.append(p)


##
#   @brief Print final results.
##
def print_results():
    print "\n\n" + bcolors.BOLD + bcolors.UNDERLINE + bcolors.YELLOW + \
        "******* Results ********\n" + bcolors.ENDC
    print bcolors.OKGREEN + "[ Succeded ]: {%s / %s}" \
        % (len(results['success']), results['num_tests'])
    for passed in results['success']:
        print "- " + passed
    print "\n" + bcolors.FAIL + "[ Failed ]: {%s / %s}" \
        % (len(results['failed']), results['num_tests'])
    for failed in results['failed']:
        print "- " + failed


##
#   @brief Parse input paths and export found test files.
#
#   @param args Arguments.
#   @param paths Path directories to look for test files.
#
def get_test_files(args, paths):
    tests = []
    if args.testClass and args.testClass in testClasses:
        for path in paths:
            # Find all the tests corresponding to given test class
            files = [ f for f in listdir(path) if isfile(join(path, f)) \
                and testClassMatch[args.testClass] in f ]
    elif args.fileName == None:
        for path in paths:
            # Find and run all the tests located under python_tests dirrectory
            files = [ f for f in listdir(path) if isfile(join(path, f)) ]
    else:
        files = args.fileName  # Input test files from arguments

    for f in files:
        f = f.replace('default_tests/', '')
        clean_file = f.split('.')
        if len(clean_file) == 1:
            pass
        elif clean_file[1] == "pyc" or clean_file[0] == "template" :
            continue
        tests.append(clean_file[0])

    return tests


##
#   @brief Load and execute input given tests.
#
#   @param tests List of tests to execute.
#   @param numCalls Number of executions.
#   @param threaded If true the execution is handled by threads.
#
##
def execute_tests_all(tests, numCalls, threaded):
    if threaded:
        core = "Parallel"
        threads = []
    else:
        core = "Serial"

    ## ------------------------- Print Header -------------------------- ##
    count = 1
    print "\033[0;33m"
    print "***************************"
    print "     RAPP Platfrom Tests   "
    print "***************************"
    print bcolors.BOLD + bcolors.OKBLUE + bcolors.UNDERLINE
    print "* Parameters:" + bcolors.ENDC
    print "-- Number of Executions for each given test: [%s] " % numCalls
    print "-- %s execution" % core
    print bcolors.BOLD + bcolors.OKBLUE + bcolors.UNDERLINE
    print "* Tests to Execute:" + bcolors.ENDC
    for test in tests:
        print "%s] %s x%s" % (count, test, numCalls)
        count += 1
    # print "\033[0;33m***************************\033[1;32m"
    time.sleep(1)
    ## ---------------------------------------------------------------- ##

    # -- Loop throug test files to be executed -- #
    for test in tests:
        module = importlib.import_module(test)

        for i in range(0, numCalls):
            if threaded:
                _id = 'thread#' + str(i +1)
                thread = Thread(target=execute, args=(module, _id, ))
                thread.start()
                threads.append(thread)
            else:
                _id = 'sequential#' + str(i + 1)
                execute(module, _id)

    if threaded:
        # Wait for all threads to complete
        for t in threads:
            t.join()



##
#   @brief Main.
##
def main():
    global __pkgDir__
    rospack = rospkg.RosPack()
    # Load this package absolute path.
    __pkgDir__ = rospack.get_path('rapp_testing_tools')
    # Load default and user given external directory paths to test files.
    testPaths = load_tests_paths()
    # Append above loaded paths to system path variable.
    append_to_system_path(testPaths)
    # Parse input arguments
    args = parse_args()

    numCalls = args.numCalls
    threaded = args.threaded
    # Load test files to execute based on user input arguments
    testFiles = get_test_files(args, testPaths)
    # Execute loaded tests. Use input number-of-calls and threaded arguments.
    execute_tests_all(testFiles, numCalls, threaded)
    # Print final results.
    print_results()
