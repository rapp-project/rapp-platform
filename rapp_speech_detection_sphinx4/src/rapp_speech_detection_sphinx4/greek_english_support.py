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

# Authors: Athanassios Kintsakis, Aris Thallas, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, aris.thallas@{iti.gr, gmail.com}, etsardou@iti.gr


import re

from english_support import *
from greek_support import *

## @class GreekSupport
# @brief Allows the creation of configuration files for Greek Sphinx speech recognition
#
# Also supports multilanguage words (English/Greek) by utilizing
# english_support.EnglishSupport
class GreekEnglishSupport(EnglishSupport, GreekSupport):

  ## Performs initializations
  def __init__(self):

    # Initialize LanguageSupport
    super(GreekEnglishSupport, self).__init__()

    # TODO: Split the rapp_sphinx4_java_libraries package into libraries and
    # language models
    # NOTE: This does not exist yet
    #self._greek_dictionary = self._language_models_url + \
        #"/englishPack/cmudict-en-us.dict"


  ## Separates English from Greek words
  #
  # @param words      [list::string] The set of words to be identified
  # @param grammar    [list::string] The Sphinx grammar parameter
  # @param sentences  [list::string] The Sphinx sentences parameter
  #
  # @return english_words     [list::string] The set of English words
  # @return english_grammar   [list::string] The set of English grammar
  # @return english_sentences [list::string] The set of English sentences
  # @return greek_words       [list::string] The set of Greek words
  # @return greek_grammar     [list::string] The set of Greek words
  # @return greek_sentences   [list::string] The set of Greek words
  def _separateEngGrWords(self, words, grammar, sentences):

    english_words = []
    english_grammar = []
    english_sentences = []
    greek_words = []
    greek_grammar = []
    greek_sentences = []

    for word in words:
      if re.match('[a-zA-Z\-]', word):
        RappUtilities.rapp_print( "English word: " + str(word) )
        english_words.append( word )
      else:
        RappUtilities.rapp_print( "Greek word: " + str(word) )
        greek_words.append( word )

    for word in grammar:
      if re.match('[a-zA-Z\-]', word):
        RappUtilities.rapp_print( "English grammar: " + str(word) )
        english_grammar.append( word )
      else:
        RappUtilities.rapp_print( "Greek grammar: " + str(word) )
        greek_grammar.append( word )

    for word in sentences:
      if re.match('[a-zA-Z\-]', word):
        RappUtilities.rapp_print( "English sentence: " + str(word) )
        english_sentences.append( word )
      else:
        RappUtilities.rapp_print( "Greek sentence: " + str(word) )
        greek_sentences.append( word )

    return [ english_words, english_grammar, english_sentences, greek_words, \
    greek_grammar, greek_sentences ]

  ## Computes the Limited Greek/English Configuration
  #
  # @param words      [list::string] The set of words to be identified
  # @param grammar    [list::string] The Sphinx grammar parameter
  # @param sentences  [list::string] The Sphinx sentences parameter
  #
  # @return limited_sphinx_configuration [dictionary] The Limited Greek/English configuration
  # @return englified_to_greek_dict      [dictionary] A dictionary to transform the englified greek words to actual greek words
  def getLimitedVocebularyConfiguration(self, words, grammar, sentences):

    # Separate English from Greek words
    [ english_words, english_grammar, english_sentences, \
      greek_words, greek_grammar, greek_sentences ] = \
      self._separateEngGrWords( words, grammar, sentences )

    # Get phonemes for Greek words and dictionary for Englified->Greek mapping
    [englified_phonems_dict, englified_to_greek_dict] = \
        self._transformWords( greek_words )

    # Append english words to Englified->Greek mapping dictionary
    for word in english_words:
      englified_to_greek_dict.update( {word: word} )

    # Get phonemes for English words
    english_phonem_dict = self._english_support.getWordPhonemes( english_words )

    # Englify Greek grammar and sentences
    englified_grammar = self._englify_words(greek_grammar)
    englified_sentences = self._englify_words(greek_sentences)


    # Join English and Greek processed files
    final_phoneme_dict = english_phonem_dict
    final_phoneme_dict.update(englified_phonems_dict)
    final_sentences = englified_sentences + english_sentences
    final_grammar = english_grammar + englified_grammar

    try:
        limited_sphinx_configuration = \
            self._vocabulary.createConfigurationFiles( \
              final_phoneme_dict, final_grammar, final_sentences \
            )
    except RappError as e:
        raise RappError(e.value)

    return [limited_sphinx_configuration, englified_to_greek_dict]
