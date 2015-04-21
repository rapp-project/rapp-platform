#!/usr/bin/env python
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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr


import rospy
import sys
import rospkg
import mmap

from limited_vocabulary_creator import *

class GreekSupport:

  def __init__(self):
    
    self.generic_sphinx_configuration = {}
    self.limited_sphinx_configuration = {}

    self.vocabulary = LimitedVocabularyCreator()
    rospack = rospkg.RosPack()
    # TODO: Split the rapp_sphinx4_java_libraries package into libraries and
    # language models
    self.greek_dictionary = rospack.get_path('rapp_sphinx4_java_libraries')   
    # NOTE: This does not exist yet
    self.greek_dictionary = self.greek_dictionary + \
        "/englishPack/cmudict-en-us.dict"
    
    self.sphinx4_jars = rospack.get_path('rapp_sphinx4_java_libraries')   
    self.sphinx4_class_path = rospack.get_path('sphinx4_wrapper')   
    
    jar_path = ".:" + self.sphinx4_jars + "/sphinx4-core-1.0-SNAPSHOT.jar:" \
            + self.sphinx4_class_path + "/src"

    # Grammar is dummy here..
    # NOTE: Fix this according to the generic Greek model
    self.generic_sphinx_configuration = { \
      'jar_path' : jar_path, \
      'configuration_path' : self.sphinx4_jars+"/greekPack/default.config.xml", \
      'acoustic_model' : self.sphinx4_jars + "/englishPack/en-us/", \
      'grammar_name' : 'hello', \
      'grammar_folder' : self.sphinx4_jars+"/greekPack/", \
      'dictionary' : self.sphinx4_jars + "/englishPack/cmudict-en-us.dict", \
      'language_model' : self.sphinx4_jars + "/englishPack/en-us.lm.dmp", \
      'grammar_disabled' : True
      }
    
    # Open the generic english dictionary file
    # NOTE: Fix this according to the Greek generic dictionary
    #try:
      #self.english_dict_file = open(self.english_dictionary, 'r')
    #except IOError:
      #print "English dictionary could not be opened!"
    # Create a mapping of the file
    #self.english_dict_mapping = mmap.mmap(self.english_dict_file.fileno(), 0, \
        #access = mmap.ACCESS_READ)

    self.configureLetters()

  def configureLetters(self):
    self.phonems = {}
    self.phonems[u'ου'] = 'UW '
    self.phonems[u'ού'] = 'UW '
    self.phonems[u'μπ'] = 'B '
    self.phonems[u'ντ'] = 'D '
    self.phonems[u'γκ'] = 'G ' #?
    self.phonems[u'γγ'] = 'G ' #?
    self.phonems[u'τσ'] = 'CH ' #?
    self.phonems[u'τζ'] = 'JH ' #?
    
    self.two_digit_letters = {}
    self.two_digit_letters[u'αι'] = 'EH '
    self.two_digit_letters[u'αί'] = 'EH '
    self.two_digit_letters[u'ει'] = 'IH '
    self.two_digit_letters[u'εί'] = 'IH '
    self.two_digit_letters[u'οι'] = 'IH '
    self.two_digit_letters[u'οί'] = 'IH '
    self.two_digit_letters[u'υι'] = 'IH '
    self.two_digit_letters[u'υί'] = 'IH '

    self.s_specific_rules = {}
    self.s_specific_rules[u'σγ'] = 'Z W '
    self.s_specific_rules[u'σβ'] = 'Z V '
    self.s_specific_rules[u'σδ'] = 'Z DH '

    self.letters = {}
    self.letters[u'α'] = 'AA ' # when AE?
    self.letters[u'ά'] = 'AA '
    self.letters[u'β'] = 'V '
    self.letters[u'γ'] = 'W '
    self.letters[u'δ'] = 'DH '
    self.letters[u'ε'] = 'EH '
    self.letters[u'έ'] = 'EH '
    self.letters[u'ζ'] = 'Z '
    self.letters[u'η'] = 'IH '
    self.letters[u'θ'] = 'TH '
    self.letters[u'ι'] = 'IH '
    self.letters[u'ί'] = 'IH '
    self.letters[u'ϊ'] = 'IH '
    self.letters[u'ΐ'] = 'IH '
    self.letters[u'κ'] = 'K '
    self.letters[u'λ'] = 'L '
    self.letters[u'μ'] = 'M '
    self.letters[u'ν'] = 'N '
    self.letters[u'ξ'] = 'K S '
    self.letters[u'ο'] = 'OW '
    self.letters[u'ό'] = 'OW '
    self.letters[u'π'] = 'P '
    self.letters[u'ρ'] = 'R '
    self.letters[u'σ'] = 'S '
    self.letters[u'τ'] = 'T '
    self.letters[u'υ'] = 'IH '
    self.letters[u'ύ'] = 'IH '
    self.letters[u'ϋ'] = 'IH ' 
    self.letters[u'ΰ'] = 'IH '
    self.letters[u'φ'] = 'F '
    self.letters[u'χ'] = 'HH '
    self.letters[u'ψ'] = 'P S '
    self.letters[u'ω'] = 'OW '
    self.letters[u'ώ'] = 'OW '

  def transformWords(self, words):
    enhanced_words = {}
    print self.letters
    for word in words:
      initial_word = word
      enhanced_words[word] = []
      
      # check phonems
      for ph in self.phonems:
        initial_word.replace(ph.encode('utf-8'), self.phonems[ph])
      # check two-digit letters
      for let in self.two_digit_letters:
        initial_word.replace(let.encode('utf-8'), self.two_digit_letters[let])
      # check specific rules
      for sr in self.s_specific_rules:
        initial_word.replace(sr.encode('utf-8'), self.s_specific_rules[sr])
      # check the rest of the letters
      for l in self.letters:
        initial_word.replace(l.encode('utf-8'), self.letters[l])
      
      enhanced_words[word] = initial_word.split(' ')
    return enhanced_words

  def getLimitedVocebularyConfiguration(self, words, grammar, sentences):
    enhanced_words = {}
    # NOTE: The following should work with the Greek generic dictionary
    #for word in words:
      #index = self.english_dict_mapping.find("\n" + word + " ") 
      #if  index == -1:
        #print "ERROR: Word " + word + " does not exist in the English Dictionary"
      #else:
        #self.english_dict_file.seek(index + 1) # +1 because of the extra \n
        #line = self.english_dict_file.readline()
        #line = line[:-1] # to erase the \n
        #split_line = line.split(" ")
        #enhanced_words[split_line[0]] = []
        #for i in range(1, len(split_line)):
          #enhanced_words[split_line[0]].append(split_line[i])

    print self.transformWords(words)
    blah = blah
    
    self.limited_sphinx_configuration = \
        self.vocabulary.createConfigurationFiles(enhanced_words, grammar, sentences)

    return self.limited_sphinx_configuration
  
  def getGenericConfiguration(self):
    return self.generic_sphinx_configuration
