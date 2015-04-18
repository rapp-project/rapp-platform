#!/usr/bin/env python

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

class Sphinx4Wrapper: 
 
  # Helper function for getting input from Sphinx
  def readLine(self, print_line = False):
    line = self.p.stdout.readline()
    if print_line:
      print line
    return line

  # Perform Sphinx4 initialization. For now it is initialized with the 
  # reduced Greek model
  def initializeSphinx(self, \
      jar_path, \
      configuration_path, \
      acoustic_model, \
      grammar_name, \
      grammar_folder, \
      dictionary, \
      language_model, \
      grammar_enabled):

    self.p = subprocess.Popen(["java", "-cp", jar_path, "Sphinx4"], \
            stdin = subprocess.PIPE, stdout = subprocess.PIPE)

    self.p.stdin.write("configurationPath#" + configuration_path)
    self.readLine(True)
    
    self.p.stdin.write("acousticModel#" + acoustic_model)
    self.readLine(True)

    self.p.stdin.write("grammarName#" + grammar_name + "#" + grammar_folder) 
    self.readLine(True)

    self.p.stdin.write("dictionary#" + dictionary)
    self.readLine(True)
    
    self.p.stdin.write("languageModel#" + language_model)
    self.readLine(True)

    self.p.stdin.write("disableGrammar#\r\n")
    self.readLine(True)

    self.p.stdin.write("forceConfiguration#\r\n")
    self.readLine(True)

  # Performs the speech recognition and returns a list of words
  def performSpeechRecognition(self, audio_file):
    self.p.stdin.write("start\r\n")
    self.p.stdin.write("audioInput#" + audio_file + "\r\n")
    start_time = time.time()
    self.readLine()
    words = []
    while(True):
      line = self.readLine()
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

