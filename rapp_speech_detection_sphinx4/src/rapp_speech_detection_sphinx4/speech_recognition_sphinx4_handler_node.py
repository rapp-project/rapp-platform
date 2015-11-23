#!/usr/bin/env python
# -*- encode: utf-8 -*-

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

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

    self._lock = threading.Condition()
    self._threadCounter = 0

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

    self._lock.acquire()
    self._threadCounter += 1
    if self._threadCounter > self._threads:
      self._lock.wait()

    # Search for available Sphinx with similar configuration
    for proc in self._availableProcesses:
      if proc['running'] == False and \
         proc['configuration_hash'] == request_hash:

        proc['running'] = True
        self._lock.release()
        total_res = proc['sphinx'].speechRecognitionBatch( req )

        self._lock.acquire()
        proc['running'] = False
        self._threadCounter -= 1
        if self._threadCounter >= self._threads:
          self._lock.notify()
        self._lock.release()

        return total_res

    # Search for available Sphinx
    for proc in self._availableProcesses:
      if proc['running'] == False:

        proc['configuration_hash'] = request_hash
        proc['running'] = True

        self._lock.release()
        total_res = proc['sphinx'].speechRecognitionBatch( req )

        self._lock.acquire()
        proc['running'] = False
        self._threadCounter -= 1
        if self._threadCounter >= self._threads:
          self._lock.notify()
        self._lock.release()

        return total_res

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

