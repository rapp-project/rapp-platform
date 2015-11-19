#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Authors: Aris Thallas
# contact: aris.thallas@{iti.gr, gmail.com}

import rospy
import sys
import time
import hashlib
import threading

from speech_recognition_sphinx4 import *


from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4TotalSrv,
  SpeechRecognitionSphinx4TotalSrvResponse
  )


class SpeechRecognitionSphinx4HandlerNode():

  def __init__(self):

    self._threads = \
        rospy.get_param("rapp_speech_detection_sphinx4_threads")

    # Initialize Speech Recognition Processes
    self._availableProcesses = [{
      'sphinx': SpeechRecognitionSphinx4(), \
      'running': False, \
      'configuration_hash': 0\
      } for i in range(self._threads)]

    self._lock = threading.Lock()

    self.serv_batch_topic = \
        rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
    if(not self.serv_batch_topic):
      rospy.logerror("Sphinx4 Speech detection batch topic param not found")

    # Initialize Service
    self.speech_recognition_batch_service = rospy.Service( \
        self.serv_batch_topic, SpeechRecognitionSphinx4TotalSrv, \
        self.handleSpeechRecognition)

  def handleSpeechRecognition(self, req):

    total_res = SpeechRecognitionSphinx4TotalSrvResponse()

    request_hash = self.calculateRequestHash( req )
    #rospy.loginfo(request_hash)

    #rospy.logerr( "Locking mutex" )
    self._lock.acquire()
    #rospy.logerr( "Mutex acquired" )

    while True:

      # Search for available Sphinx with similar configuration
      for proc in self._availableProcesses:
        if proc['running'] == False and \
           proc['configuration_hash'] == request_hash:

          #rospy.logerr( "Found process with same configuration" )
          #rospy.logerr( proc )

          proc['running'] = True
          self._lock.release()
          total_res = proc['sphinx'].speechRecognitionBatch( req )

          proc['running'] = False

          return total_res

      # Search for available Sphinx
      for proc in self._availableProcesses:
        if proc['running'] == False:

          proc['configuration_hash'] = request_hash
          proc['running'] = True

          #rospy.logerr( "Found process" )
          #rospy.logerr( proc )

          self._lock.release()
          total_res = proc['sphinx'].speechRecognitionBatch( req )

          proc['running'] = False

          return total_res
      time.sleep(0.1)
      #rospy.logerr( "Waiting for process" )

    #rospy.logerr( "Could not find available processes" )
    #self._lock.release()
    #return total_res

  def calculateRequestHash(self, req):
    hash_object = hashlib.sha1()
    hash_object.update( req.language )
    for word in req.words:
      hash_object.update( word )
    for gram in req.grammar:
      hash_object.update( gram )
    for sent in req.sentences:
      hash_object.update( sent )
    return hash_object.hexdigest()


if __name__ == "__main__":
  rospy.init_node('SpeechRecognitionSphinx4')
  SpeechRecognitionSphinx4HandlerNode = SpeechRecognitionSphinx4HandlerNode()
  rospy.spin()

