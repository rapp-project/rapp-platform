#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Authors: Aris Thallas
# contact: aris.thallas@iti.gr

import yaml
import hashlib

from global_parameters import GlobalParams

from rapp_utilities import RappUtilities

## @class SphinxConfigurationParams
# @brief Contains the parameters required for the Sphinx configuration
class SphinxConfigurationParams():

  ## @brief Initializes an empty configuration (constructor)
  def __init__(self, configuration = None):

    ## Contains global Sphinx parameters
    #
    # (see global_parameters.GlobalParams)
    self._globalParams = GlobalParams()

    ## The language of the request
    self._language = 'el'
    ## The words to be identified
    self._words = []
    ## Sphinx grammar attribute
    self._grammar = []
    ## Sphinx sentence attribute
    self._sentences = []

    if configuration != None:
      self._readConfigurationYaml( configuration )

  ## @brief Change attributes to those specified by the request
  #
  # @param params [rapp_platform_ros_communications::SpeechDetectionSphinx4Wrapper::SpeechRecognitionSphinx4TotalSrvRequest] The service request
  def makeEqualToRequest(self, params):
    self._language = params.language
    self._words = params.words
    self._grammar = params.grammar
    self._sentences = params.sentences

  ## @brief Change attributes to those specified by the Class instance
  #
  # @param params [SphinxConfigurationParams] The class instance
  def makeEqualToInstance(self, instance):
    self._language = instance._language
    self._words = instance._words
    self._grammar = instance._grammar
    self._sentences = instance._sentences

  ## @brief Read the configuration from configuration yaml
  #
  # @param confName [string] The name of the requested configuration
  def _readConfigurationYaml(self, confName):
    RappUtilities.rapp_print( "Reading preconfiguration: " + confName )
    yamlStream = open( self._globalParams._sphinx_preconf, 'r' )
    yamlFile = yaml.load(yamlStream)
    if confName not in yamlFile['configurations']:
      RappUtilities.rapp_print('Wrong configuration name provided. Leaving ' + \
          'default values', 'ERROR')
      return

    tempConf = SphinxConfigurationParams()

    # Read configuration language
    yamlLang = yamlFile['configurations'][confName]['language']
    if yamlLang != None:
      tempConf._language = yamlLang.encode('utf-8')
    else:
      RappUtilities.rapp_print('Configuration language not provided. ' + \
          'Leaving default values', 'ERROR')
      return

    # Read configuration words
    yamlWords = yamlFile['configurations'][confName]['words']
    if yamlWords != None:
      tempConf._words = [word.encode('utf-8') for word in yamlWords]
    else:
      RappUtilities.rapp_print('Configuration words not provided. ' + \
          'Leaving default values', 'ERROR')
      return

    # Read configuration sentences
    yamlSent = yamlFile['configurations'][confName]['sentences']
    if yamlSent != None:
      tempConf._sentences = [sentence.encode('utf-8') for sentence in yamlSent]
    else:
      RappUtilities.rapp_print('Configuration sentences not provided. ' + \
          'Leaving default values', 'ERROR')
      return

    # Read configuration grammar
    yamlGram = yamlFile['configurations'][confName]['grammar']
    if yamlGram != None:
      tempConf._grammar = [grammar.encode('utf-8') for grammar in yamlGram]
    else:
      tempConf._grammar = []

    # Copy temporary values to self
    self.makeEqualToInstance( tempConf )

    RappUtilities.rapp_print( 'Language: ' + str(self._language) )
    RappUtilities.rapp_print( 'Words: ' + str(self._words) )
    for word in self._words:
      RappUtilities.rapp_print( 'word: ' + str(word) )
    RappUtilities.rapp_print( 'Grammar: ' + str(self._grammar))
    RappUtilities.rapp_print( 'Sentences: ' + str(self._sentences) )



  ## @brief Checks if a SphinxConfigurationParams instance equals self
  #
  # @param params [SphinxConfigurationParams] The class instance
  # @param status [bool] True if configurations match
  def equalsRequest(self, params):
    if ( self._language == params.language and \
      self._words == params.words and \
      self._grammar == params.grammar and \
      self._sentences == params.sentences
    ):
      return True
    else:
      return False

  ## @brief Calculates the configuration's sha1 hash
  #
  # Hash is used to identify common request configurations for proper
  # subprocess selection.
  # (Requests with common requests do not require reconfiguration reducing
  # computation time)
  #
  # @return hexdigest [string] The hash digest containing only hexadecimal digits
  def getHash(self):
    hash_object = hashlib.sha1()
    hash_object.update( self._language )
    for word in self._words:
      hash_object.update( word )
    for gram in self._grammar:
      hash_object.update( gram )
    for sent in self._sentences:
      hash_object.update( sent )
    return hash_object.hexdigest()
