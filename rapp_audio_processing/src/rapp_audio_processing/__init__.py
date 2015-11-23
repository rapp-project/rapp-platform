#! /usr/bin/env python
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

# Authors: Manos Tsardoulias, Aris Thallas
# contact: etsardou@iti.gr, aris.thallas@{iti.gr, gmail.com}

from rapp_audio_processing import AudioProcessing
from rapp_detect_silence import DetectSilence
from rapp_energy_denoise import EnergyDenoise
from rapp_sox_denoise import SoxDenoise
from rapp_utilities import Utilities
from rapp_set_noise_profile import SetNoiseProfile
from rapp_transform_audio import TransformAudio
