#!/usr/bin/python

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

import os
import rospkg

## @class GlobalParams
# @brief Contains global Sphinx parameters
class GlobalParams:
  def __init__(self):
    self.rospack = rospkg.RosPack()

    ## Java libraries path
    self._sphinx_jar_files_url = self.rospack.get_path("rapp_sphinx4_java_libraries")
    ## Sphinx package path
    self._sphinx_package_url = self.rospack.get_path("rapp_speech_detection_sphinx4")
    ## Language models path
    self._language_models_url = self.rospack.get_path("rapp_sphinx4_language_models")
    ## Temporary language models path
    self._tmp_language_models_url = os.path.join( os.environ['HOME'], \
        'rapp_platform_files/rapp_speech_recognition_sphinx4/' )
    ## Noise profiles path
    self._noise_profiles_url = self.rospack.get_path("rapp_sphinx4_noise_profiles")
    ## Acoustic models path
    self._acoustic_models_url = self.rospack.get_path("rapp_sphinx4_acoustic_models")
    self._acoustic_models_url += "/english_acoustic_model"
    ## Sphinx jar file path
    self._sphinx_jar_file = 'sphinx4-core-1.0-20150630.174404-9.jar'
    ## True if Sphinx is allowed output on STDIN, STDERR
    self._allow_sphinx_output = False
    ## IPC socket HOST parameter
    self._socket_host = '127.0.0.1'
