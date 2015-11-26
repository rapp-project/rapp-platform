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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr


import rospy
import sys
import tempfile
import atexit
import shutil
import os

from global_parameters import GlobalParams
from rapp_exceptions import RappError

from rapp_tools import *

## @class LimitedVocabularyCreator
# @brief Creates temporary configuration files for the input limited vocabulary
class LimitedVocabularyCreator(GlobalParams):

  ## Performs initializations
  def __init__(self):
    GlobalParams.__init__(self)

    if not os.path.exists( self._tmp_language_models_url ):
      rospy.logwarn( "Language temporary directory does not exist. Path: " + \
          self._tmp_language_models_url )
      os.makedirs(self._tmp_language_models_url)

    self.languages_package = tempfile.mkdtemp( prefix='tmp_language_pack_', dir = self._tmp_language_models_url )

    atexit.register(shutil.rmtree, self.languages_package)


    self.sphinx_configuration = { \
      'jar_path' : ".:" + self._sphinx_jar_files_url + \
            "/" + self._sphinx_jar_file + ":" \
            + self._sphinx_package_url + "/src", \
      'configuration_path' : self._language_models_url + "/greekPack/default.config.xml", \
      'acoustic_model' : self._acoustic_models_url, \
      'grammar_name' : '', \
      'grammar_folder' : '', \
      'dictionary' : '', \
      'language_model' : '', \
      'grammar_disabled' : True
      }


  ## Creates temporary configuration files for the input limited vocabulary
  #
  # The 'words' input argument is of the form:
  # words = {
  #           'word1_en_chars': [phonem1, phonem2,...],
  #           'word2_en_chars': [phonem1, phonem2,...]
  #           ...
  #         }
  #
  # @param words      [list::string] The set of words to be identified
  # @param grammar    [list::string] The Sphinx grammar parameter
  # @param sentences  [list::string] The Sphinx sentences parameter
  #
  # @return conf   [dictionary] The final configuration
  # @return status [string] Either the error (string) or True (bool)
  def createConfigurationFiles(self, words, grammar, sentences):
    tmp_configuration = self.sphinx_configuration

    rapp_print( "Creating configuration files with parameters:" )
    rapp_print( "Words: " + str(words) )
    rapp_print( "Sentences: " + str(sentences) )
    rapp_print( "Grammar: " + str(grammar) )
    # Create custom dictionary file
    tmp_configuration['dictionary'] = os.path.join( self.languages_package, \
        'custom.dict' )
    custom_dict = open(tmp_configuration['dictionary'], 'w')
    for word in words:
      tmp_line = word
      for phoneme in words[word]:
        tmp_line += " " + phoneme.replace('-', '').replace('_', '')
      custom_dict.write(tmp_line + '\n')
    custom_dict.close()

    # Check grammar
    if len(grammar) == 0:
      tmp_configuration['grammar_disabled'] = True
    else:
      tmp_configuration['grammar_disabled'] = False
    tmp_configuration['grammar_name'] = 'custom'
    tmp_configuration['grammar_folder'] = self.languages_package
    custom_grammar = open(os.path.join( tmp_configuration['grammar_folder'], \
        tmp_configuration['grammar_name']) + '.gram', 'w')
    custom_grammar.write('#JSGF V1.0;\n')
    custom_grammar.write("grammar " + tmp_configuration['grammar_name'] + ';\n')
    counter = 1
    #custom_grammar.write("public <cmd1>=(")
    for i in range(0, len(grammar)):
      gram = grammar[i]
      # check is all words in grammar exist in words
      gram_words = gram.split(" ")
      for gw in gram_words:
          if gw not in words and gram not in words:
              raise RappError('Word ' + gw + ' is not in words but\
                      exists in grammar')

      custom_grammar.write("public <cmd" + str(counter) + ">=" + "\"" + gram + "\";\n")
      #custom_grammar.write("\"" + gram + "\"")
      #if i == len(grammar) - 1:
          #custom_grammar.write(");")
      #else:
          #custom_grammar.write(" | ")
      counter += 1
    custom_grammar.close()

    # Fix sentences / language model
    # Check sentences: All words must exist in sentences
    # Continue with the sentences setup
    tmp_configuration['language_model'] = os.path.join( self.languages_package, \
      "sentences.lm.bin" )
    custom_sentences = open(self.languages_package + '/sentences.txt', 'w')
    if len(sentences) != 0:
      for sent in sentences:
        # Split sentence
        sent_words = sent.split(" ")
        for sw in sent_words:
            if sw not in words and sent not in words:
                raise RappError('Word ' + sw + ' is not in words but\
                      exists in a sentence')

        custom_sentences.write("<s> " + sent + " </s>\n")
    else:
      for word in words:
        custom_sentences.write("<s> " + word + " </s>\n")
    custom_sentences.close()

    # Run script to fix the language model
    rapp_print( "Sphinx: Creating language model files\n" )
    if self._allow_sphinx_output == True:
        bash_file = self._language_models_url + "/greekPack/run.sh"
        bash_command = "cp " + bash_file + " " + self.languages_package + \
            " && cd " + self.languages_package + " && bash run.sh"
    else:
        bash_file = self._language_models_url + "/greekPack/run_silent.sh"
        bash_command = "cp " + bash_file + " " + self.languages_package + \
            " && cd " + self.languages_package + " && bash run_silent.sh"

    os.system(bash_command)

    return tmp_configuration
