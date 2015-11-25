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

from global_parameters import GlobalParams

## @class SphinxConfigurationParams
# @brief Contains the parameters required for the Sphinx configuration
class SphinxConfigurationParams(GlobalParams):

  ## @brief Initializes an empty configuration (constructor)
  def __init__(self):
      GlobalParams.__init__(self)

      ## The language of the request
      self._language = 'el'
      ## The words to be identified
      self._words = []
      ## Sphinx grammar attribute
      self._grammar = []
      ## Sphinx sentence attribute
      self._sentences = []


  ## @brief Checks if a SphinxConfigurationParams instance equals self
  #
  # @param params [SphinxConfigurationParams] The class instance
  # @param status [bool] True if configurations match
  def equals(self, params):
    if ( self._language == params.language and \
      self._words == params.words and \
      self._grammar == params.grammar and \
      self._sentences == params.sentences
    ) :
      return True
    else:
      return False
