#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import timeit
import argparse

__path__ = os.path.dirname(os.path.realpath(__file__))

## ------ Access the RappCloud python module ------- ##
user = os.getlogin()
module_path = '/home/' + user + '/rapp_platform_catkin_ws/src/rapp-platform/hop_services'\
   +  '/utilities/python'
sys.path.append(module_path)
from RappCloud import *
## ------------------------------------------------##


def test_face_detection(rappCloud):
    fileUri = __path__  + '/Lenna.png'
    startT = timeit.default_timer()
    response = rappCloud.face_detection(fileUri)
    endT = timeit.default_timer()
    elapsedT = endT - startT
    
    print "Face-Detection invocation epalsed time ---> [\033[1;31m%s\033[0m]" % elapsedT

    # -- Define valid results -- #
    valid_faces_up_left = [ {'y': 200.0, 'x': 212.0, 'z': 0.0} ]
    valid_faces_down_right = [ {'y': 379.0, 'x': 391.0, 'z': 0.0} ]
    # -------------------------- #

    faces_up_left = response['faces_up_left']
    faces_down_right = response['faces_down_right']
    error = response['error']
    #print faces_up_left
    #print faces_down_right
    #print error

    # --- Evaluating results --- #
    if error == '0':
        print "\033[0;33mface-detection service returned without error\033[0m: [%s]" % error
        if valid_faces_up_left == faces_up_left:
            print "\033[0;34mValid faces_up_left result: \033[0m [%s]" % faces_up_left
        else:
            print "\033[1;31mInalid faces_up_left result: \033[0m [%s]" % faces_up_left

        if valid_faces_down_right == faces_down_right:
            print "\033[0;34mValid faces_down_right result: \033[0m [%s]" % faces_down_right
        else:
            print "\033[1;31mInalid faces_down_right result: \033[0m [%s]" % faces_up_left
    else:
        print "\033[1;31m[ERROR] ---------> %s\033[0m" % error
    # -------------------------- #
    return elapsedT

 
def test_qr_detection(rappCloud):
    fileUri = __path__  + '/qr_code_rapp.jpg'
    startT = timeit.default_timer()
    response = rappCloud.qr_detection(fileUri)
    endT = timeit.default_timer()
    elapsedT = endT - startT
    
    print "QR-Detection invocation epalsed time ---> [\033[1;31m%s\033[0m]" % elapsedT

    # -- Define valid results -- #
    valid_qr_centers = [ {'y': 165.0, 'x': 165.0, 'z': 0.0} ]
    # -------------------------- #

    qr_centers = response['qr_centers']
    error = response['error']
    #print qr_centers
    #print error

    # --- Evaluating results --- #
    if error == '0':
        print "\033[0;33mface-detection service returned without error\033[0m: [%s]" % error
        if valid_qr_centers == qr_centers:
            print "\033[0;34mValid qr_centers result: \033[0m [%s]" % qr_centers
        else:
            print "\033[0;34mInvalid qr_centers result: \033[0m [%s]" % qr_centers
    else:
        print "\033[1;31m[ERROR] ---------> %s\033[0m" % error
    # -------------------------- #
    return elapsedT
 

def test_set_denoise_profile(rappCloud):
    fileUri = __path__ + '/denoise_source.wav'
    audio_source = 'nao_wav_1_ch'
    user = 'klpanagi'

    startT = timeit.default_timer()
    response = rappCloud.set_denoise_profile(fileUri, audio_source, user)
    endT = timeit.default_timer()
    elapsedT = endT - startT
    
    print "Set-Denoise-Profile invocation epalsed time ---> [\033[1;31m%s\033[0m]" % elapsedT

    error = response['error']
    #print response

    # --- Evaluating results --- #
    if error == '0':
        print "\033[0;33mSet-Denoise-Profile service returned without error\033[0m: [%s]" % error
    else:
        print "\033[1;31m[ERROR] ---------> %s\033[0m" % error
    # -------------------------- #
    return elapsedT


def test_speech_detection_sphinx4(rappCloud):
    language = 'gr'
    fileUri = __path__ + '/nai-oxi-test.wav'
    audio_source = 'nao_wav_1_ch'
    words = []
    words.append(u'ναι')
    words.append(u'οχι')
    sentences = [u'ναι',u'οχι']
    grammar = []
    user = 'klpanagi'

    startT = timeit.default_timer()
    response = rappCloud.speech_detection_sphinx4(language, audio_source, words, sentences, grammar, fileUri, user)
    endT = timeit.default_timer()
    elapsedT = endT - startT
    
    print "[Speech-Detection-Sphix4] invocation epalsed time ---> [\033[1;31m%s\033[0m]" % elapsedT

    # -- Define valid results -- #
    valid_words_found = [u'ναι', u'οχι']
    # -------------------------- #

    words_found = response['words']
    error = response['error']
    #print response['words']

    # --- Evaluating results --- #
    if error == '0':
        print "\033[0;33m[Speech-Detection-Sphinx4\033[0m] service returned without error: [%s]" % error
        if valid_words_found == words_found:
            print "\033[0;34mValid words_found result: \033[0m [%s]" % words_found
        else:
            print "\033[0;34mInvalid words_found result: \033[0m [%s]" % words_found
    else:
        print "\033[1;31m[ERROR] ---------> %s\033[0m" % error
    # -------------------------- #
    return elapsedT


def line_seperator(symbol, length, color):
    line = color
    for i in range(0, length):
        line += symbol
    line += '\033[0m'
    return line



def parse_args():
    parser = argparse.ArgumentParser(description='RAPP Platform front-end hop-service invocation tests')
    parser.add_argument('-i','--name', help='Service to call. \033[1;34mall==all\033[0m',\
            dest='serviceName', action='store', nargs=1, type=str)

    parser.add_argument('-n', '--', dest='numCalls', action='store', \
            nargs=1, help='Number of simultaneous service calls')

    args =  parser.parse_args( ) # Parse the given arguments
    srvName = args.serviceName[0]
    numCalls = args.numCalls[0]

    ##### Console Output #####
    print '\n\033[1;32;41mInput Arguments:\033[0m'
    print line_seperator('=', 50, '')
    print '[Service Name]:  ', srvName
    print '[Number of Simultaneous Calls]:  ', numCalls
    ###########################
    return {'srvName': srvName, 'numCalls': int(numCalls)}

def main():
    rappCloud = RappCloud()
    sum_execTime = 0
    count = 1
    platServices = rappCloud.get_platform_services()
    print "\033[1;32;41mAvailable RAPP Platform Servicces to call:\033[0m"
    print line_seperator('=', 50, '\033[0m')
    for services in platServices:
        print str(count) + ']:  ' + str(services)
        count += 1

    #rappCloud.call_service(service)
    
    args = parse_args()

    if args['srvName'] == 'speech_detection_sphinx4':
        for i in range(0, args['numCalls']):
            print "\n\033[1;33mCall #%s:" % (i + 1)
            print line_seperator('-', 10, '\033[1;33m')
            #print "--------------------\033[0m"
            sum_execTime += test_speech_detection_sphinx4(rappCloud)
    elif args['srvName'] == 'face_detection':
        for i in range(0, args['numCalls']):
            print "\n\033[1;33mCall #%s:" % (i + 1)
            print line_seperator('-', 10, '\033[1;33m')
            #print "--------------------\033[0m"
            sum_execTime += test_face_detection(rappCloud)
    elif args['srvName'] == 'qr_detection':
        for i in range(0, args['numCalls']):
            print "\n\033[1;33mCall #%s:" % (i + 1)
            print line_seperator('-', 10, '\033[1;33m')
            #print "--------------------\033[0m"
            sum_execTime += test_qr_detection(rappCloud)
    elif args['srvName'] == 'set_denoise_profile':
        for i in range(0, args['numCalls']):
            print "\n\033[1;33mCall #%s:" % (i + 1)
            print line_seperator('-', 10, '\033[1;33m')
            #print "--------------------\033[0m"
            sum_execTime += test_set_denoise_profile(rappCloud)
    else:
        print "\033[1;34;42mWrong Input Service Name!!!!! Exiting...\033[0m"
        sys.exit(1)
            


    
    print "\n======================================================================\n"
    print "Execution Time for \033[1;31m%s\033[0m x {\033[1;31m%s\033[0m} service calls: [\033[36m%s\033[0m]" \
            % (args['numCalls'], args['srvName'], sum_execTime) 

    avg = 1 * (sum_execTime / args['numCalls'])

    print "Average execution time ------> [\033[33m%s\033[0m]" % avg


if __name__ == "__main__":
    try:
        main()
    except:
        pass

