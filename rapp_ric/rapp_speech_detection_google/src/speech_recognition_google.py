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

from rapp_platform_ros_communications.srv import (
  SpeechToTextSrv,
  SpeechToTextSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from std_msgs.msg import (
  String
  )

class SpeechToTextGoogle:

  def __init__(self):
    # Speech recognition service published
    self.serv_topic = rospy.get_param("rapp_speech_detection_google_topic")
    if(not self.serv_topic):
        rospy.logerror("Speech detection topic param not found")
    self.serv = rospy.Service(self.serv_topic, \
        SpeechToTextSrv, self.speech_to_text_callback)

  # The service callback  
  def speech_to_text_callback(self, req):
    # Getting the results in order to parse them
    transcripts = self.speech_to_text(req.filename.data)
    # The alternative results
    alternatives = transcripts['result'][0]['alternative']
    
    res = SpeechToTextSrvResponse()

    # If alternatives is 0 returns void response
    if len(alternatives) > 0:
      # The first alternative is Google's suggestion
      words = alternatives[0]['transcript'].split(" ")
      for w in words:
        res.words = res.words + [String(data=w)]
      # Google provides the confidence for the first suggestion
      res.confidence.data = alternatives[0]['confidence']

      # Google API may return other alternatives without confidence
      for alt in alternatives[1:]:
        sam = StringArrayMsg()
        words = alt['transcript'].split(" ")
        for w in words:
          sam.s = sam.s + [String(data=w)]
        res.alternatives = res.alternatives + [sam]
    else:
      res.confidence.data = 0
    return res
   

  #NOTE The audio file should be flac to work.
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
    # Google returns one empty result for some reason here. Removing it..
    index = data.find("}")
    data = data[index + 1:]
    jsdata = json.loads(data)
    return jsdata

if __name__ == "__main__":
  rospy.init_node('speech_to_text_ros_node')
  speech_to_text_node = SpeechToTextGoogle()
  rospy.spin()


