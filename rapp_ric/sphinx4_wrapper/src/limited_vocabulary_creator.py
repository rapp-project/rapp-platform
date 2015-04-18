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


import rospy
import sys
import rospkg
import os

class LimitedVocabularyCreator:

  def __init__(self):
    self.sphinx_configuration = { \
      'jar_path' : '', \
      'configuration_path' : '', \
      'acoustic_model' : '', \
      'grammar_name' : '', \
      'grammar_folder' : '', \
      'dictionary' : '', \
      'language_model' : '', \
      'grammar_disabled' : True
      }

    rospack = rospkg.RosPack()
    self.languages_package = rospack.get_path('rapp_sphinx4_java_libraries')   
    self.languages_package += "/tmp_language_pack/"
    if not os.path.exists(self.languages_package):
      os.makedirs(self.languages_package)


  # Creates temporary configuration files for the input limited vocabulary
  # The 'words' input argument is of the form:
  # words = {
  #           'word1_en_chars': [phonem1, phonem2,...],
  #           'word2_en_chars': [phonem1, phonem2,...]
  #           ...
  #         }
  def createConfigurationFiles(self, words):
    
    # Create custom dictionary file
    self.sphinx_configuration['dictionary'] = self.languages_package + 'custom.dict'
    custom_dict = open(self.sphinx_configuration['dictionary'], 'w')
    for word in words:
      tmp_line = word
      for phoneme in words[word]:
        tmp_line += " " + phoneme
      custom_dict.write(tmp_line + '\n')
    custom_dict.close()
    print words
