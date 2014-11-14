#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import httplib
import json
import sys

#TESTCASE: python speech_recognition_google.py ../../../test_auxiliary_files/test.flac

def speech_to_text(audio):
  url = "www.google.com"
  language = "en-US" 
  # NOTE - Thats a general usage key. They may disable it in the future.
  key = "AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw"
  path = "/speech-api/v2/recognize?lang=" + language + "&key=" + key
  headers = { "Content-type": "audio/x-flac; rate=22050" };
  params = {"xjerr": "1", "client": "chromium"}
  conn = httplib.HTTPSConnection(url)
  conn.request("POST", path, audio, headers)
  response = conn.getresponse()
  data = response.read()
  print data # something is wrong with the json format
  #jsdata = json.loads(data)
  #return jsdata

if __name__ == "__main__":
  if len(sys.argv) != 2 or "--help" in sys.argv:
    print "Usage: stt.py <flac-audio-file>"
    sys.exit(-1)
  else:
    with open(sys.argv[1], "r") as f:
      speech = f.read()
      text = speech_to_text(speech)
      print text
