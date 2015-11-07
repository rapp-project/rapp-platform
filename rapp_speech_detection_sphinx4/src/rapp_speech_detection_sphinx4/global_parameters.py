#!/usr/bin/python

import os
import rospkg

class GlobalParams:
  def __init__(self):
    self.rospack = rospkg.RosPack()

    self.sphinx_jar_files_url = self.rospack.get_path("rapp_sphinx4_java_libraries")
    self.sphinx_package_url = self.rospack.get_path("rapp_speech_detection_sphinx4")
    self.language_models_url = self.rospack.get_path("rapp_sphinx4_language_models")
    self.tmp_language_models_url = os.path.join( os.environ['HOME'], \
        'rapp_platform_files/rapp_speech_recognition_sphinx4/' )
    self.noise_profiles_url = self.rospack.get_path("rapp_sphinx4_noise_profiles")
    self.acoustic_models_url = self.rospack.get_path("rapp_sphinx4_acoustic_models")
    self.acoustic_models_url += "/english_acoustic_model"
    self.sphinx_jar_file = 'sphinx4-core-1.0-20150630.174404-9.jar'
    self.allow_sphinx_output = False
    self.socket_host = '127.0.0.1'
