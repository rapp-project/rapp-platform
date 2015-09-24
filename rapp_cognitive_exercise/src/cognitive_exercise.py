#!/usr/bin/env python

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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr

import rospy
import MySQLdb as mdb
import sys
import xml.etree.ElementTree as ET

import calendar
import time

from datetime import datetime
from os.path import expanduser
from test_selector import TestSelector
from recordUserCognitiveTestPerformance import RecordUserCognitiveTestPerformance

from rapp_platform_ros_communications.srv import (
  testSelectorSrv,  
  testSelectorSrvResponse,
  createOntologyAliasSrv,
  createOntologyAliasSrvRequest,
  createOntologyAliasSrvResponse,
  recordUserCognitiveTestPerformanceSrv,
  recordUserCognitiveTestPerformanceSrvResponse
  )
  
from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  ) 
    
from std_msgs.msg import ( 
  String 
  ) 

class CognitiveExercise: 
  
  def __init__(self):   
    #tblUser services launch
    self.serv_topic = rospy.get_param("rapp_cognitive_exercise_chooser_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_cognitive_exercise_chooser_topic not found")   
    self.serv=rospy.Service(self.serv_topic, testSelectorSrv, self.chooserDataHandler) 
    
    self.serv_topic = rospy.get_param("rapp_cognitive_exercise_record_user_cognitive_test_performance_topic")
    if(not self.serv_topic):
      rospy.logerror("rapp_cognitive_exercise_record_user_cognitive_test_performance_topic not found")   
    self.serv=rospy.Service(self.serv_topic, recordUserCognitiveTestPerformanceSrv, self.recordUserCognitiveTestPerformanceDataHandler) 
                    
  #tblUser callbacks    
  def chooserDataHandler(self,req):     
    res = testSelectorSrvResponse()
    it = TestSelector()    
    res=it.chooserFunction(req) 
    return res    
    
  def recordUserCognitiveTestPerformanceDataHandler(self,req):     
    res = recordUserCognitiveTestPerformanceSrvResponse()
    it = RecordUserCognitiveTestPerformance()    
    res=it.recordPerformance(req) 
    return res   
    
if __name__ == "__main__": 
  rospy.init_node('CognitiveExercise')
  CognitiveExerciseNode = CognitiveExercise() 
  rospy.spin()
