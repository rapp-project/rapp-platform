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
import sys
import subprocess
import time
import rospkg

from rapp_platform_ros_communications.srv import (
  Sphinx4WrapperSrv,
  Sphinx4WrapperSrvResponse
  )
    
from std_msgs.msg import ( 
  String 
  ) 

class Sphinx4Wrapper: 
  
  def __init__(self):       
    #self.serv=rospy.Service('ric/sphinx4_wrapper', Sphinx4WrapperSrv, self.sphinx4DataHandler)
    self.serv_topic = rospy.get_param("rapp_speech_detection_sphinx4_topic")
    if(not self.serv_topic):
      rospy.logerror("Sphinx4 Speech detection topic param not found")
    self.serv = rospy.Service(self.serv_topic, Sphinx4WrapperSrv, self.sphinx4DataHandler)
    rospack = rospkg.RosPack()
    self.sphinx4_jars = rospack.get_path('rapp_sphinx4_java_libraries')   
    self.sphinx4_class_path = rospack.get_path('sphinx4_wrapper')   
    total_path = ".:" + self.sphinx4_jars + "/sphinx4-core-1.0-SNAPSHOT.jar:" \
            + self.sphinx4_class_path + "/src"
    self.p = subprocess.Popen(["java", "-cp", total_path, "Sphinx4"], \
            stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    self.p.stdin.write("configurationPath#"+self.sphinx4_jars+"/greekPack/default.config.xml\r\n")
    line = self.p.stdout.readline()
    print line
    
    self.p.stdin.write("acousticModel#"+self.sphinx4_jars+"/acoustic_model\r\n")
    line = self.p.stdout.readline()
    print line

    self.p.stdin.write("grammarName#hello#"+self.sphinx4_jars+"/greekPack/\r\n")
    line = self.p.stdout.readline()
    print line

    self.p.stdin.write("dictionary#" + self.sphinx4_jars + "/greekPack/custom.dict\r\n")
    print "dictionary#" + self.sphinx4_jars + "/greekPack/custom.dict"
    line = self.p.stdout.readline()
    print line
    
    self.p.stdin.write("languageModel#"+self.sphinx4_jars+"/greekPack/sentences.lm.dmp\r\n")
    line = self.p.stdout.readline()
    print line
 
 #viewUsersRobotsApps callbacks    
  def sphinx4DataHandler(self,req):     
    res = Sphinx4WrapperSrvResponse()
    #res=self.fetchData(req,"usersrobotsapps")    
    #p = subprocess.Popen(["java", "-cp", "/home/thanos/NetBeansProjects/MyClass/dist/MyClass.jar"], stdin=subprocess.PIPE)
    #p = subprocess.Popen(['java', '/home/thanos/NetBeansProjects/MyClass/build/classes//myclass.MyClass'], stdin=subprocess.PIPE)
    #p = subprocess.Popen(["java","-cp", "/home/thanos/local_catkin_workspaces/catkin_ws/src/rapp-platform/rapp_ric/sphinx4_wrapper/src/","MyClass"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    
    self.p.stdin.write("start\r\n")
    self.p.stdin.write("audioInput#" + req.path.data + "\r\n")
    #p.stdin.write("x\r\n") # this line will not be printed into the file
    print("out")
    start_time = time.time()
    line = self.p.stdout.readline()
    
    while(True):
      line = self.p.stdout.readline()
      if(len(line)>0):
        if(line[0]=="#"):
          res.words.data=res.words.data+"\n"+line
          #break
        if(line=="stopPython\n"):
          #res.words.data=line
          break
        print line  
      if (time.time() - start_time > 10):
        res.words.data="Time out error"
        break
      
      
    #print line
    #res.words.data=line
    return res;  
  #viewUsersRobotsApps
    
if __name__ == "__main__": 
  rospy.init_node('Sphinx4Wrapper')
  Sphinx4WrapperNode = Sphinx4Wrapper() 
  rospy.spin()
