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

class Sphinx4Wrapper: 
 
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
  def performSpeechRecognition(self, audio_file):
    # NOTE: Check if file exists here!
    
    # If it is an .ogg file (from NAO) recode it into .wav
    new_audio_file = audio_file

    # TODO: Check for channels / ending

    #if audio_file.endswith(".ogg"):
    audio_file_base = os.path.basename(audio_file)
    new_audio_file_base = "new_" + audio_file_base[:audio_file_base.find('.')] + ".wav"
    # NOTE: Using global command for now
    bash_command = "cd " + os.path.dirname(audio_file) + " && "\
        "sox " + os.path.dirname(audio_file) + "/" + audio_file_base + " "\
        "-r 16000 -c 1 " +\
        os.path.dirname(audio_file) + "/" + new_audio_file_base
    print "####### " + bash_command
    os.system(bash_command)
    bash_command = "cd " + os.path.dirname(audio_file) + " && "\
        "sox " + os.path.dirname(audio_file) + "/" + new_audio_file_base + \
        " " + os.path.dirname(audio_file) + "/final_" + new_audio_file_base + \
        " noisered " + "/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform-supplementary-material/rapp_sphinx4_java_libraries/recordings/nao_noise_prof" + " 0.1"
    print '############## ' + bash_command
    os.system(bash_command)

    new_audio_file = os.path.dirname(audio_file) + "/final_" + new_audio_file_base

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

