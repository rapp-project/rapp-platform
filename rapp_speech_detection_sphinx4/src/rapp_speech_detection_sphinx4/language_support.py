#!/usr/bin/env python

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


import rospy
import sys
import mmap

from global_parameters import GlobalParams
from rapp_exceptions import RappError
from limited_vocabulary_creator import *
from rapp_tools import *

## @class LanguageSupport
# @brief Allows the creation of configuration files for Sphinx speech recognition
class LanguageSupport(object):

  ## Performs initializations
  def __init__(self):
    ## Contains global Sphinx parameters
    #
    # (see global_parameters.GlobalParams)
    self._globalParams = GlobalParams()

    ## The limited vocabulary creator
    #
    # Instantiates limited_vocabulary_creator.LimitedVocabularyCreator
    self._vocabulary = LimitedVocabularyCreator()

    jar_path = ".:" + self._globalParams._sphinx_jar_files_url + "/" + \
        self._globalParams._sphinx_jar_file + ":" + \
        self._globalParams._sphinx_package_url + "/src"

    # Grammar is dummy here..
    ## The generic Sphinx configuration
    #
    # @note Check acoustic model!!
    self._generic_sphinx_configuration = { \
      'jar_path' : jar_path, \
      'configuration_path' : self._globalParams._language_models_url + \
                                      "/greekPack/default.config.xml", \
      'acoustic_model' : self._globalParams._acoustic_models_url, \
      'grammar_name' : 'hello', \
      'grammar_folder' : self._globalParams._language_models_url + \
                                                    "/greekPack/", \
      'dictionary' : self._globalParams._language_models_url + \
                            "/englishPack/cmudict-en-us.dict", \
      'language_model' : self._globalParams._language_models_url + \
                                      "/englishPack/en-us.lm.bin", \
      'grammar_disabled' : True
      }


  ## Computes the Limited English Configuration
  #
  # @param words      [list::string] The set of words to be identified
  # @param grammar    [list::string] The Sphinx grammar parameter
  # @param sentences  [list::string] The Sphinx sentences parameter
  #
  # @return limited_sphinx_configuration [dictionary] The Limited configuration
  # @return englified_to_lang_dict       [dictionary] A dictionary to transform the englified words to the words in the proper language
  def getLimitedVocebularyConfiguration(self, words, grammar, sentences):
    raise NotImplementedError("Child classes should explicitly override this function")

  ## Returns the Generic English Configuration
  #
  # @return #_generic_sphinx_configuration [dictionary] The Generic English configuration
  def getGenericConfiguration(self):
    return self._generic_sphinx_configuration
