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

import calendar
import time

from datetime import datetime
from os.path import expanduser

from rapp_platform_ros_communications.srv import (
  fetchDataSrv,
  fetchDataSrvResponse,
  writeDataSrv,
  writeDataSrvResponse,
  deleteDataSrv,
  deleteDataSrvResponse,
  updateDataSrv,
  updateDataSrvResponse
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
    self.serv=rospy.Service(self.serv_topic, fetchDataSrv, self.tblUserFetchDataHandler) 
                
  #tblUser callbacks    
  def tblUserFetchDataHandler(self,req):     
    res = fetchDataSrvResponse()
    tmp=expanduser('~')
    
    res.trace.append(tmp)
    timestamp = str(int(time.time()))

    edibleTime=datetime.now().strftime('%d/%m/%Y %H:%M:%S')
    res.trace.append(edibleTime)
    res.trace.append(timestamp)
    #res=self.fetchData(req,"tblUser")
    return res    
    
if __name__ == "__main__": 
  rospy.init_node('CognitiveExercise')
  CognitiveExerciseNode = CognitiveExercise() 
  rospy.spin()
