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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr


import rospy
import sys
import subprocess
import time
import rospkg

from greek_transliteration import *
from sphinx4_wrapper import *

from rapp_platform_ros_communications.srv import (
  Sphinx4WrapperSrv,
  Sphinx4WrapperSrvResponse
  )
    
from std_msgs.msg import ( 
  String 
  ) 

class SpeechRecognitionSphinx4: 
 
  # Constructor performing initializations
  def __init__(self):    

    self.sphinx4 = Sphinx4Wrapper()
    self.greek = GreekTransliteration()
    
    self.serv_topic = rospy.get_param("rapp_speech_detection_sphinx4_topic")
    
    if(not self.serv_topic):
      rospy.logerror("Sphinx4 Speech detection topic param not found")
    
    self.serv = rospy.Service(self.serv_topic, Sphinx4WrapperSrv, self.sphinx4DataHandler)
    
    rospack = rospkg.RosPack()

    self.sphinx4_jars = rospack.get_path('rapp_sphinx4_java_libraries')   
    self.sphinx4_class_path = rospack.get_path('sphinx4_wrapper')   
    
    total_path = ".:" + self.sphinx4_jars + "/sphinx4-core-1.0-SNAPSHOT.jar:" \
            + self.sphinx4_class_path + "/src"

    self.sphinx4.initializeSphinx(\
            total_path,\
            self.sphinx4_jars+"/greekPack/default.config.xml\r\n",\
            self.sphinx4_jars+"/acoustic_model\r\n",\
            "hello",\
            self.sphinx4_jars+"/greekPack/\r\n",\
            self.sphinx4_jars + "/greekPack/custom.dict\r\n",\
            self.sphinx4_jars+"/greekPack/sentences.lm.dmp\r\n",\
            False)
 
  # Service callback for handling speech recognition
  def sphinx4DataHandler(self,req):     
    res = Sphinx4WrapperSrvResponse()
    words = self.sphinx4.performSpeechRecognition(req.path.data)   
    
    for word in words:
      res.words.append(word)
   
    return res;  

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionSphinx4')
  SpeechRecognitionSphinx4Node = SpeechRecognitionSphinx4()
  rospy.spin()
