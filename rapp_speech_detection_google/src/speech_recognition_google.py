#!/usr/bin/env python2
# -*- coding: utf-8 -*-

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
import os

from rapp_platform_ros_communications.srv import (
  SpeechToTextSrv,
  SpeechToTextSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from rapp_exceptions import RappError

class SpeechToTextGoogle:

  def __init__(self):
    # Speech recognition service published
    self.serv_topic = rospy.get_param("rapp_speech_detection_google_detect_speech_topic")
    if(not self.serv_topic):
        rospy.logerror("Speech detection google topic param not found")

    self.threads = rospy.get_param("rapp_speech_detection_google_threads")
    if not self.threads:
        rospy.logerror("Speech detection google threads param not found")
        self.threads = 0

    self.serv = rospy.Service(self.serv_topic, \
        SpeechToTextSrv, self.speech_to_text_callback)
    for i in range(0, self.threads):
        rospy.Service(self.serv_topic + '_' + str(i), \
            SpeechToTextSrv, self.speech_to_text_callback)

  # The service callback  
  def speech_to_text_callback(self, req):
    
    res = SpeechToTextSrvResponse()

    # Getting the results in order to parse them
    try:
        transcripts = self.speech_to_text(req.filename)
        print transcripts
    except RappError as e:
        res.error = e.value
        return res

    if len(transcripts['result']) == 0:
        return res

    # The alternative results
    alternatives = transcripts['result'][0]['alternative']
    res = SpeechToTextSrvResponse()

    # If alternatives is 0 returns void response
    if len(alternatives) > 0:
      # The first alternative is Google's suggestion
      words = alternatives[0]['transcript'].split(" ")
      for w in words:
        res.words = res.words + [w]
      # Google provides the confidence for the first suggestion
      res.confidence.data = alternatives[0]['confidence']

      for alt in alternatives[1:]:
        sam = StringArrayMsg()
        words = alt['transcript'].split(" ")
        for w in words:
          sam.s = sam.s + [w]
        res.alternatives = res.alternatives + [sam]
    else:
      res.confidence.data = 0
    return res
   

  #NOTE The audio file should be flac to work.
  def speech_to_text(self, file_path):
    
    # Check if file exists
    if not os.path.isfile(file_path):
        raise RappError("Error: file " + file_path + ' not found')

    # Check if file is flac. If not convert it
    new_file_path = file_path
    if "wav" in file_path or "ogg" in file_path:
        new_file_path = file_path + '.flac'
        command = 'flac -f --channels=1 --sample-rate=16000 '\
                + file_path + ' -o ' + new_file_path
        if os.system(command):
            raise RappError("Error: flac command malfunctioned. File path was"\
                    + file_path)
        
    with open(new_file_path, "r") as f:
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
    initial_data = data
    # Google returns one empty result for some reason here. Removing it..
    index = data.find("}")
    data = data[index + 1:]
    if data == '\n':
        # Returned nothing.. something went wrong
        data = initial_data
    jsdata = json.loads(data)

    # Remove the flac if needed
    if new_file_path != file_path:
        command = 'rm -f ' + new_file_path
        if os.system(command):
            raise RappError("Error: Removal of temporary flac file malfunctioned")
    return jsdata

if __name__ == "__main__":
  rospy.init_node('speech_to_text_ros_node')
  speech_to_text_node = SpeechToTextGoogle()
  rospy.spin()


