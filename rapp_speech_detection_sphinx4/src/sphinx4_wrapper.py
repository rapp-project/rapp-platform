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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr

import sys
import subprocess
import time
import os
from global_parameters import GlobalParams
import rospy

from rapp_platform_ros_communications.srv import(
    AudioProcessingDenoiseSrv,
    AudioProcessingDenoiseSrvRequest,
    AudioProcessingDetectSilenceSrv,
    AudioProcessingDetectSilenceSrvRequest
    )

class Sphinx4Wrapper(GlobalParams): 
 
  def __init__(self):
    GlobalParams.__init__(self)
    self.denoise_topic = rospy.get_param("rapp_audio_processing_denoise_topic")
    self.energy_denoise_topic = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")
    self.detect_silence_topic = \
        rospy.get_param("rapp_audio_processing_detect_silence_topic")


    if(not self.denoise_topic):
      rospy.logerror("Audio processing denoise topic not found")
    if(not self.energy_denoise_topic):
      rospy.logerror("Audio processing energy denoise topic not found")
    if(not self.detect_silence_topic):
      rospy.logerror("Audio processing detect silence topic not found")

    self.denoise_service = rospy.ServiceProxy(\
              self.denoise_topic, AudioProcessingDenoiseSrv)
    self.energy_denoise_service = rospy.ServiceProxy(\
              self.energy_denoise_topic, AudioProcessingDenoiseSrv)
    self.detect_silence_service = rospy.ServiceProxy(\
              self.detect_silence_topic, AudioProcessingDetectSilenceSrv)


  # Helper function for getting input from Sphinx
  def readLine(self, print_line = False):
    line = self.p.stdout.readline()
    if print_line:
      print line
    return line

  # Perform Sphinx4 initialization. For now it is initialized with the 
  # reduced Greek model
  def initializeSphinx(self, conf):

    self.p = subprocess.Popen(["java", "-cp", conf['jar_path'], \
            "Sphinx4"], stdin = subprocess.PIPE, stdout = subprocess.PIPE)

    self.p.stdin.write("configurationPath#" + conf['configuration_path'] + '\r\n')
    self.readLine(True)
    
    self.p.stdin.write("acousticModel#" + conf['acoustic_model'] + '\r\n')
    self.readLine(True)

    self.p.stdin.write("grammarName#" + conf['grammar_name'] + "#" + \
            conf['grammar_folder'] + '\r\n') 
    self.readLine(True)

    self.p.stdin.write("dictionary#" + conf['dictionary'] + '\r\n')
    self.readLine(True)
    
    self.p.stdin.write("languageModel#" + conf['language_model'] + '\r\n')
    self.readLine(True)

    if(conf['grammar_disabled']):
      self.p.stdin.write("disableGrammar#\r\n")
    else:
      self.p.stdin.write("enableGrammar#\r\n")
    self.readLine(True)

    self.p.stdin.write("forceConfiguration#\r\n")
    self.readLine(True)

  def configureSphinx(self, conf):
    self.p.stdin.write("configurationPath#" + conf['configuration_path'] + '\r\n')
    self.readLine(True)
    self.p.stdin.write("acousticModel#" + conf['acoustic_model'] + '\r\n')
    self.readLine(True)
    self.p.stdin.write("grammarName#" + conf['grammar_name'] + "#" + \
            conf['grammar_folder'] + '\r\n') 
    self.readLine(True)
    self.p.stdin.write("dictionary#" + conf['dictionary'] + '\r\n')
    self.readLine(True)
    self.p.stdin.write("languageModel#" + conf['language_model'] + '\r\n')
    self.readLine(True)
    if(conf['grammar_disabled']):
      self.p.stdin.write("disableGrammar#\r\n")
    else:
      self.p.stdin.write("enableGrammar#\r\n")
    self.readLine(True)
    self.p.stdin.write("forceConfiguration#\r\n")
    self.readLine(True)


  # Performs the speech recognition and returns a list of words
  def performSpeechRecognition(self, audio_file, audio_source, user):
    # Check if path exists
    if os.path.isfile(audio_file) == False:
      return ["Error: Something went wrong with the local audio storage",\
              "Requested path: " + audio_file]

    
    # Keep extra audio files that need erasing
    audio_to_be_erased = []

    # If it is an .ogg file (from NAO) recode it into .wav
    next_audio_file = audio_file
    prev_audio_file = next_audio_file

    audio_file_folder = os.path.dirname(audio_file)
    if audio_file_folder[-1] != "/":
      audio_file_folder += "/"

    # Check that the audio_source is legit
    if audio_source not in ["headset", "nao_ogg", "nao_wav_4_ch", "nao_wav_1_ch"]:
      return ["Error: Audio source unrecognized"]

    # Transform audio to 16kHz, mono if needed
    
    if audio_source == "nao_ogg": # Needs only ogg->wav & denoising
      print "Audio source = NAO ogg"
      next_audio_file += ".wav"
      command = "sox " + prev_audio_file + " " + next_audio_file
      print "RAPP: " + command
      os.system(command)      
      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file
    elif audio_source == "nao_wav_4_ch": # Needs conversion to mono + 16KHz
      print "Audio source = NAO wav 4 channels"
      next_audio_file = next_audio_file + "_mono.wav"
      command = "sox " + prev_audio_file + " -r 16000 -c 1 " + next_audio_file
      print "RAPP: " + command
      os.system(command)
      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file

    # Check if denoising is needed
    if audio_source == "nao_ogg" or \
        audio_source == "nao_wav_1_ch" or\
        audio_source == "nao_wav_4_ch":

      next_audio_file = prev_audio_file + "_denoised.wav"
      den_request = AudioProcessingDenoiseSrvRequest()
      den_request.audio_file = prev_audio_file
      den_request.denoised_audio_file = next_audio_file
      den_request.audio_type = audio_source
      den_request.user = user
      den_request.scale = 0.15
      den_response = self.denoise_service(den_request)
      if den_response.success != "true":
        return ["Error:" + den_response.success]
      audio_to_be_erased.append(next_audio_file)

    # Detect silence
    silence_req = AudioProcessingDetectSilenceSrvRequest()
    silence_req.audio_file = prev_audio_file
    silence_res = self.detect_silence_service(silence_req)
    if silence_res.silence == "true":
      return ["Error: No speech detected. RSD = " + str(silence_res.level)]

    # Perform energy denoising as well
    energy_denoise_req = AudioProcessingDenoiseSrvRequest()
    next_audio_file = prev_audio_file + "_energy_denoised.wav"
    energy_denoise_req.audio_file = prev_audio_file
    energy_denoise_req.denoised_audio_file = next_audio_file
    energy_denoise_res = self.energy_denoise_service(energy_denoise_req)
    if energy_denoise_res.success != "true":
      return ["Error:" + energy_denoise_res.success]
    audio_to_be_erased.append(next_audio_file)

    new_audio_file = next_audio_file
    self.p.stdin.write("start\r\n")
    self.p.stdin.write("audioInput#" + new_audio_file + "\r\n")
    start_time = time.time()
    self.readLine()
    words = []
    while(True):
      line = self.readLine()
      print line
      if(len(line)>0):
        if(line[0]=="#"):
          stripped_down_line = line[1:-1].split(" ")
          for word in stripped_down_line:
            words.append(word)
        if(line=="stopPython\n"):
          break
      if (time.time() - start_time > 10):
        words.append("Error: Time out error")
        break
    
    directory = "/tmp/rapp_platform_files/rapp_speech_recognition_sphinx4/" + user
    if not os.path.isdir(directory):
      os.makedirs(directory)

    if len(audio_to_be_erased) != 0:
      clean_file = audio_to_be_erased[-1].split("/")[-1]
      clean_file_or = audio_to_be_erased[0].split("/")[-1]
      command = "cp " + audio_to_be_erased[-1] + " " + directory + "/" + clean_file
      os.system(command)
      command = "cp " + audio_to_be_erased[-1] + " " + directory + "/original_" + clean_file
      os.system(command)

    for f in audio_to_be_erased:
      command = "rm " + f
      print "------------------" + command
      os.system(command)

    return words

