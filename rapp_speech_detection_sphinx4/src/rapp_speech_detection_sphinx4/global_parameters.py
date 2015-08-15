#!/usr/bin/python

import rospkg

class GlobalParams:
  def __init__(self):
    self.rospack = rospkg.RosPack()
    
    self.sphinx_jar_files_url = self.rospack.get_path("rapp_sphinx4_java_libraries")
    self.sphinx_package_url = self.rospack.get_path("rapp_speech_detection_sphinx4")
    self.language_models_url = self.rospack.get_path("rapp_sphinx4_language_models")
    self.noise_profiles_url = self.rospack.get_path("rapp_sphinx4_noise_profiles")
    self.acoustic_models_url = self.rospack.get_path("rapp_sphinx4_acoustic_models")
    self.acoustic_models_url += "/english_acoustic_model"

