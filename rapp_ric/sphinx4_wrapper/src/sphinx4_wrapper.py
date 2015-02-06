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


import rospy
import sys
import subprocess

from rapp_platform_ros_communications.srv import (
  Sphinx4WrapperSrv,
  Sphinx4WrapperSrvResponse
  )
    
from std_msgs.msg import ( 
  String 
  ) 

class Sphinx4Wrapper: 
  
  def __init__(self):       
    self.serv=rospy.Service('ric/sphinx4_wrapper', Sphinx4WrapperSrv, self.sphinx4DataHandler)
                
      
  #viewUsersRobotsApps callbacks    
  def sphinx4DataHandler(self,req):     
    res = Sphinx4WrapperSrvResponse()
    #res=self.fetchData(req,"usersrobotsapps")    
    #p = subprocess.Popen(["java", "-cp", "/home/thanos/NetBeansProjects/MyClass/dist/MyClass.jar"], stdin=subprocess.PIPE)
    #p = subprocess.Popen(['java', '/home/thanos/NetBeansProjects/MyClass/build/classes//myclass.MyClass'], stdin=subprocess.PIPE)
    #p = subprocess.Popen(["java","-cp", "/home/thanos/local_catkin_workspaces/catkin_ws/src/rapp-platform/rapp_ric/sphinx4_wrapper/src/","MyClass"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    p = subprocess.Popen(["java","-cp", ".:/home/thanos/NetBeansProjects/Sphinx4Maven/target/classes/sphinx4-core-1.0-SNAPSHOT.jar:/home/thanos/NetBeansProjects/Sphinx4Maven/target/classes/sphinx4-data-1.0-SNAPSHOT.jar:/home/thanos/NetBeansProjects/Sphinx4Maven/target/classes/","Sphinx4"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    #p.stdin.write("First line\r\n")
    #p.stdin.write("Second line\r\n")
    #p.stdin.write("x\r\n") # this line will not be printed into the file
    print("out")
    line = p.stdout.readline()
    while(line != "x\n"):
      print line
      line = p.stdout.readline()

    res.words.data="return nothing"
    return res;  
  #viewUsersRobotsApps
    
if __name__ == "__main__": 
  rospy.init_node('Sphinx4Wrapper')
  Sphinx4WrapperNode = Sphinx4Wrapper() 
  rospy.spin()
