#!/usr/bin/python

import sys
import rospy

def rapp_print(var):
    rospy.logdebug( str(var) )
