#!/usr/bin/env python2

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
import httplib
import json
import sys

from rapp_platform_ros_communications.srv import SpeechToTextSrv

#TESTCASE: python speech_recognition_google.py ../../../test_auxiliary_files/test.flac

class SpeechToTextGoogle:

  def __init__(self):
    self.serv = rospy.Service('ric/speech_to_text_service', SpeechToTextSrv, self.speech_to_text_callback)

  def speech_to_text_callback(self, req):
    res = SpeechToTextSrcResponce()
    words = self.speech_to_text(req.filename)
    return res


  def speech_to_text(self, file_path):
    with open(file_path, "r") as f:
      speech = f.read()
    url = "www.google.com"
    language = "en-US" 
    #NOTE - Thats a general usage key. They may disable it in the future.
    key = "AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw"
    path = "/speech-api/v2/recognize?lang=" + language + "&key=" + key
    headers = { "Content-type": "audio/x-flac; rate=22050" };
    params = {"xjerr": "1", "client": "chromium"}
    conn = httplib.HTTPSConnection(url)
    conn.request("POST", path, speech, headers)
    response = conn.getresponse()
    data = response.read()
    return data # something is wrong with the json format
    #jsdata = json.loads(data)
    #return jsdata

if __name__ == "__main__":
  rospy.init_node('speech_to_text_ros_node')
  speech_to_text_node = SpeechToTextGoogle()
  rospy.spin()


