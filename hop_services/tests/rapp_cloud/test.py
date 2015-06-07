#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import timeit

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
        if valid_faces_down_right == faces_down_right:
            print "\033[0;34mValid faces_down_right result: \033[0m [%s]" % faces_down_right
    # -------------------------- #
        
    

def main():
    rappCloud = RappCloud()
    platServices = rappCloud.get_platform_services()
    #print platServices

    #service = 'speech_detection_sphinx4'
    #rappCloud.call_service(service)
    
    test_face_detection(rappCloud)


if __name__ == "__main__":
    try:
        main()
    except:
        pass

