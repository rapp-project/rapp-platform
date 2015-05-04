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
import rospkg

class Sphinx4Wrapper: 
 
  def __init__(self):
    self.rospack = rospkg.RosPack()

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
  def performSpeechRecognition(self, audio_file, audio_source):
    # Check if path exists
    if os.path.isfile(audio_file) == False:
      return ["Error: Something went wrong with the local audio storage"]
    
    # If it is an .ogg file (from NAO) recode it into .wav
    next_audio_file = audio_file
    prev_audio_file = next_audio_file

    audio_file_folder = os.path.dirname(audio_file)
    if audio_file_folder[-1] != "/":
      audio_file_folder += "/"
    next_audio_file = os.path.basename(prev_audio_file)

    # Check that the audio_source is legit
    if audio_source not in ["headset", "nao_ogg", "nao_wav_4_ch", "nao_wav_1_ch"]:
      return ["Error: Audio source unrecognized"]

    # Transform audio to 16kHz, mono if needed
    
    if audio_source == "nao_ogg": # Needs only ogg->wav & denoising
      print "Audio source = NAO ogg"
      next_audio_file += ".wav"
      command = "sox " + prev_audio_file + " " + next_audio_file
      print command
      os.system("cd " + audio_file_folder + " && " + command)      
    elif audio_source == "nao_wav_4_ch": # Needs conversion to mono + 16KHz
      print "Audio source = NAO wav 4 channels"
      next_audio_file = "mono_" + next_audio_file
      command = "sox " + prev_audio_file + " -r 16000 -c 1 " + next_audio_file
      print command
      os.system("cd " + audio_file_folder + " && " + command)

    # Check if denoising is needed
    if audio_source == "nao_ogg":
      nao_ogg_noise_profile = self.rospack.get_path("rapp_sphinx4_java_libraries")
      nao_ogg_noise_profile += "noise_profiles/noise_profile_nao_ogg"
      next_audio_file = "denoised_" + prev_audio_file
      command = "sox " + prev_audio_file + " " + next_audio_file + " noisered "\
          + nao_ogg_noise_profile + " 0.1"
      print command
      os.system("cd " + audio_file_folder + " && " + command)
    elif audio_source == "nao_wav_4_ch" or audio_source == "nao_wav_1_ch":
      nao_wav_noise_profile = self.rospack.get_path("rapp_sphinx4_java_libraries")
      nao_wav_noise_profile += "noise_profiles/noise_profile_nao_wav"
      next_audio_file = "denoised_" + prev_audio_file 
      command = "sox " + prev_audio_file + " " + next_audio_file + " noisered "\
          + nao_wav_noise_profile + " 0.1"
      print command
      os.system("cd " + audio_file_folder + " && " + command)

    new_audio_file = audio_file_folder + next_audio_file
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
        words.append("Time out error")
        break
    
    return words

