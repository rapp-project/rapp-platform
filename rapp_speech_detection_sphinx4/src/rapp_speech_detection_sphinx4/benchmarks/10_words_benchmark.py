#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr

import rospy
import sys
from os import listdir
from os.path import isfile, join
import os

from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4Srv,
  SpeechRecognitionSphinx4SrvResponse,
  SpeechRecognitionSphinx4SrvRequest,
  SpeechRecognitionSphinx4ConfigureSrv,
  SpeechRecognitionSphinx4ConfigureSrvResponse,
  SpeechRecognitionSphinx4ConfigureSrvRequest,
  SpeechRecognitionSphinx4TotalSrv,
  SpeechRecognitionSphinx4TotalSrvResponse,
  SpeechRecognitionSphinx4TotalSrvRequest,

  AudioProcessingDenoiseSrv,
  AudioProcessingDenoiseSrvResponse,
  AudioProcessingDenoiseSrvRequest
  )

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class SpeechRecognitionTester:

  def setup_two_words_voc(self):
    spreq = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spreq.language = 'el'
    spreq.words = []
    spreq.words.append(u'οχι')
    spreq.words.append(u'ναι')
    spreq.words.append(u'αρκετα')
    spreq.words.append(u'ειμαι')
    spreq.words.append(u'εισαι')
    spreq.words.append(u'ειναι')
    spreq.words.append(u'φουρνος')
    spreq.words.append(u'γιατρος')
    #spreq.words.append(u'κατσε')
    #spreq.words.append(u'χαπια')
    #spreq.words.append(u'χαπι')
    #spreq.words.append(u'ρομποτ')
    #spreq.words.append(u'στειλε')
    #spreq.words.append(u'στειλεις')
    spreq.sentences = spreq.words
    spreq.grammar = spreq.words
    return spreq

  def __init__(self):

    self.conf_sp_ser_top = \
        rospy.get_param("rapp_speech_detection_sphinx4_total_topic")
    self.denoising_service = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")

    self.conf_sp_ser = rospy.ServiceProxy(\
        self.conf_sp_ser_top,\
        SpeechRecognitionSphinx4TotalSrv)
    self.denoising = rospy.ServiceProxy(\
        self.denoising_service,\
        AudioProcessingDenoiseSrv)

    self.spreq = ""
    self.spreq = self.setup_two_words_voc()
    self.spreq.grammar = []
    self.spee_req = SpeechRecognitionSphinx4TotalSrvRequest()

    if not (len(sys.argv) == 2 or len(sys.argv) == 4):
      print "Invalid number of arguments"
      return

    if len(sys.argv) == 2:
      folder = sys.argv[1]
      files = [ f for f in listdir(folder) if isfile(join(folder, f)) ]
    else:
      folder = sys.argv[1]
      files = [sys.argv[2]]

    replays = 1
    if len(sys.argv) == 4:
      replays = int(sys.argv[3])
    for i in range(0, replays):
      [notrec, failed ] = self.run(files, folder)
      #self.run(notrec + failed, folder)

  def run(self, files, folder):
    success = 0
    total = 0
    didnt_recognize = 0
    total_recognize = 0

    failed = []
    not_recognized = []
    response = []

    for f in files:

      toprint = ""

      response = []
      if "einai_" in f or "eimai_" in f or "eisai_" in f:
        response.append( (u'ειμαι').encode('utf-8') )
        response.append( (u'εισαι').encode('utf-8') )
        response.append( (u'ειναι').encode('utf-8') )
      elif "arketa_" in f:
        response.append( (u'αρκετα').encode('utf-8') )
      elif "fournos_" in f:
        response.append( (u'φουρνος').encode('utf-8') )
      elif "giatros_" in f:
        response.append( (u'γιατρος').encode('utf-8') )
      #elif "katse_" in f:
        #response.append( (u'κατσε').encode('utf-8') )
      elif "nai_" in f:
        response.append( (u'ναι').encode('utf-8') )
      elif "oxi_" in f:
        response.append( (u'οχι').encode('utf-8') )
      #elif "xapia_" in f or "xapi_" in f:
        #response.append( (u'χαπια').encode('utf-8') )
        #response.append( (u'χαπι').encode('utf-8') )
      #elif "robot_" in f:
        #response.append( (u'ρομποτ').encode('utf-8') )
      #elif "steile_" in f or "steileis_" in f:
        #response.append( (u'στειλε').encode('utf-8') )
        #response.append( (u'στειλεις').encode('utf-8') )
      else:
        continue

      print f

      self.spee_req.language = self.spreq.language
      self.spee_req.words = self.spreq.words
      self.spee_req.grammar = self.spreq.grammar
      self.spee_req.user = 'etsardou'
      self.spee_req.sentences = self.spreq.sentences
      self.spee_req.path = folder + f
      #self.spee_req.audio_source = 'headset'
      self.spee_req.audio_source = 'nao_wav_1_ch_only_sox'
      #self.spee_req.audio_source = 'nao_wav_1_ch'

      res = self.conf_sp_ser(self.spee_req)

      toprint += os.path.basename(self.spee_req.path) + " : "
      for word in res.words:
        toprint +="'" + word + "' "

      ok = False
      while not ok:
        ok = True
        for i in range(0, len(res.words) - 1):
          if res.words[i] == res.words[i + 1]:
            del res.words[i + 1]
            ok = False
            break;

      total += 1
      if len(res.words) == 1:
        total_recognize += 1
        if res.words[0] in response:
          success += 1
          toprint = bcolors.OKGREEN + toprint + bcolors.ENDC
        else:
          failed.append(f)
          toprint = bcolors.FAIL + toprint + bcolors.ENDC
      else:
        didnt_recognize += 1
        toprint = bcolors.WARNING + toprint + bcolors.ENDC
        not_recognized.append(f)

      print toprint,
      if total_recognize != 0:
        print " / " + str(success * 1.0 / total_recognize * 100.0) + "%",
      print " / " + str(didnt_recognize*1.0 / total * 100.0) + "%"

      # For debugging purposes
      #break
    print "-----Not recognized------"
    for f in not_recognized:
      print f
    print "-----Errors------"
    for f in failed:
      print f

    print "Total files: " + str(total)
    print "Recognized: " + str(total_recognize)
    print "Not recognized: " + str(didnt_recognize)
    print "Success: " + str(success)
    print "Fails: " + str(len(failed))

    return [not_recognized, failed]


# Main function
if __name__ == "__main__":
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()
  #rospy.spin()

