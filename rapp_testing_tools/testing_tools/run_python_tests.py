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

__path__ = os.path.dirname(os.path.realpath(__file__))

# Mutex lock used when threaded.
mutex = Lock()
threaded = False

testClasses = [
    'face-detection',
    'qr-detection',
    'speech-detection',
    'speech-detection-sphinx4',
    'speech-detection-google',
    'ontology',
    'cognitive'
]

testClassMatch = {
    'face-detection' : 'face',
    'qr-detection' : 'qr',
    'speech-detection' : 'speech',
    'speech-detection-sphinx4' : 'sphinx4',
    'speech-detection-google' : 'google',
    'ontology' : 'ontology',
    'cognitive': 'cognitive'
}

results = {
    'success' : [],
    'failed' : []
}

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



def execute(module, id):
    tmp = module.RappInterfaceTest()
    [error_code, time] = tmp.execute()

    mutex.acquire(True)
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





def main():
    folder = __path__ + "/python_tests"
    sys.path.append(folder)

    # ---------------------------------------------- #
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

    # --------------------------------------------- #

    ## ------------------- Parse arguments ---------------------------- ##
    if args.testClass and args.testClass in testClasses:
        files = [ f for f in listdir(folder) if isfile(join(folder, f)) and testClassMatch[args.testClass] in f ]
    elif args.fileName == None:
        # Find and run all the tests located under python_tests dirrectory
        files = [ f for f in listdir(folder) if isfile(join(folder, f)) ]
    else:
        files = args.fileName # Input test files from arguments

    tests = []
    for f in files:

        f = f.replace('python_tests/', '')
        clean_file = f.split('.')
        if clean_file[1] != "py" or clean_file[0] == "template" :
            continue
        tests.append(clean_file[0])


    numCalls = args.numCalls
    # If threaded mode is enabled
    if args.threaded:
        threaded = True
        core = "Parallel"
        threads = []
    else:
        threaded = False
        core = "Serial"

    numTests = numCalls * len(tests)
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
            # ------------------------------------------- #
    if threaded:
        # Wait for all threads to complete
        for t in threads:
            t.join()

    ## ------- Global Results -------- ##
    print "\n\n" + bcolors.BOLD + bcolors.UNDERLINE + bcolors.YELLOW + \
        "******* Results ********\n" + bcolors.ENDC
    print bcolors.OKGREEN + "[ Succeded ]: {%s / %s}" \
        % (len(results['success']), numTests) #+ bcolors.ENDC
    for passed in results['success']:
        print "- " + passed
    print "\n" + bcolors.FAIL + "[ Failed ]: {%s / %s}" \
        % (len(results['failed']), numTests)
    for failed in results['failed']:
        print "- " + failed


if __name__ == "__main__":
    main()

