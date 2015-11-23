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

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr

import sys
import subprocess
import socket
import time
import os
from global_parameters import GlobalParams
import rospy
from rapp_tools import *

from rapp_platform_ros_communications.srv import(
    AudioProcessingDenoiseSrv,
    AudioProcessingDenoiseSrvRequest,
    AudioProcessingDetectSilenceSrv,
    AudioProcessingDetectSilenceSrvRequest,
    AudioProcessingTransformAudioSrv,
    AudioProcessingTransformAudioSrvResponse,
    AudioProcessingTransformAudioSrvRequest
    )

class Sphinx4Wrapper(GlobalParams):

  def __init__(self):
    self._conf = ''
    self.sphinxDied = False
    GlobalParams.__init__(self)
    self.denoise_topic = rospy.get_param("rapp_audio_processing_denoise_topic")
    self.energy_denoise_topic = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")
    self.detect_silence_topic = \
        rospy.get_param("rapp_audio_processing_detect_silence_topic")
    self.audio_trans_topic = \
        rospy.get_param("rapp_audio_processing_transform_audio_topic")

    if(not self.denoise_topic):
      rospy.logerror("Audio processing denoise topic not found")
    if(not self.energy_denoise_topic):
      rospy.logerror("Audio processing energy denoise topic not found")
    if(not self.detect_silence_topic):
      rospy.logerror("Audio processing detect silence topic not found")
    if(not self.audio_trans_topic):
      rospy.logerror("Audio processing transform audio topic not found")

    self.denoise_service = rospy.ServiceProxy(\
              self.denoise_topic, AudioProcessingDenoiseSrv)
    self.energy_denoise_service = rospy.ServiceProxy(\
              self.energy_denoise_topic, AudioProcessingDenoiseSrv)
    self.detect_silence_service = rospy.ServiceProxy(\
              self.detect_silence_topic, AudioProcessingDetectSilenceSrv)
    self.audio_transform_srv = rospy.ServiceProxy( \
        self.audio_trans_topic, AudioProcessingTransformAudioSrv )

  # Helper function for getting input from Sphinx
  def readLine(self):
    line = self.socket_connection.recv(1024)
    if self.allow_sphinx_output == True:
      rapp_print( line )
    return line

  # Perform Sphinx4 initialization. For now it is initialized with the
  # reduced Greek model
  def initializeSphinx(self, conf):

    rapp_print(str(conf['jar_path']))

    self.createSocket()

    if self.allow_sphinx_output == True:
        self.p = subprocess.Popen( ["java", "-cp", conf['jar_path'], "Sphinx4", str(self._sphinx_socket_PORT)] )
    else:
        try:
          from subprocess import DEVNULL
        except ImportError:
          DEVNULL = open(os.devnull, 'wb')

        self.p = subprocess.Popen( ["java", "-cp", conf['jar_path'], "Sphinx4", str(self._sphinx_socket_PORT)], \
            stdout = DEVNULL, stderr = DEVNULL )

    self.socket_connection, addr = self._sphinx_socket.accept()

    self.configureSphinx( conf )

  def createSocket(self):
    HOST = self.socket_host
    self._sphinx_socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM ) # Create Unix Socket
    self._sphinx_socket.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self._sphinx_socket.bind( (HOST, 0) )
    self._sphinx_socket_PORT = self._sphinx_socket.getsockname()[1]
    self._sphinx_socket.listen( 1 )

  def configureSphinx(self, conf):
    self._conf = conf
    self.socket_connection.sendall("configurationPath#" + conf['configuration_path'] + '\r\n')
    self.readLine()
    self.socket_connection.sendall("acousticModel#" + conf['acoustic_model'] + '\r\n')
    self.readLine()
    self.socket_connection.sendall("grammarName#" + conf['grammar_name'] + "#" + \
            conf['grammar_folder'] + '\r\n')
    self.readLine()
    self.socket_connection.sendall("dictionary#" + conf['dictionary'] + '\r\n')
    self.readLine()
    self.socket_connection.sendall("languageModel#" + conf['language_model'] + '\r\n')
    self.readLine()
    if(conf['grammar_disabled']):
      self.socket_connection.sendall("disableGrammar#\r\n")
    else:
      self.socket_connection.sendall("enableGrammar#\r\n")
    self.readLine()
    self.socket_connection.sendall("forceConfiguration#\r\n")
    self.readLine()

  def createProcessingProfile(self, audio_type):
    manipulation = {}
    manipulation['sox_transform'] = False
    manipulation['sox_channels_and_rate'] = False
    manipulation['sox_denoising'] = False
    manipulation['sox_denoising_scale'] = 0.0
    manipulation['detect_silence'] = False
    manipulation['detect_silence_threshold'] = 0.0
    manipulation['energy_denoising'] = False
    manipulation['energy_denoising_init_scale'] = 0.0

    if audio_type == "headset":
      pass
    elif audio_type == "nao_ogg":
      manipulation['sox_transform'] = True
      manipulation['sox_denoising'] = True
      manipulation['sox_denoising_scale'] = 0.15
      manipulation['detect_silence'] = True
      manipulation['detect_silence_threshold'] = 3.0
      manipulation['energy_denoising'] = True
      manipulation['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_4_ch":
      manipulation['sox_channels_and_rate'] = True
      manipulation['sox_denoising'] = True
      manipulation['sox_denoising_scale'] = 0.15
      manipulation['detect_silence'] = True
      manipulation['detect_silence_threshold'] = 3.0
      manipulation['energy_denoising'] = True
      manipulation['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_1_ch":
      manipulation['sox_denoising'] = True
      manipulation['sox_denoising_scale'] = 0.15
      manipulation['detect_silence'] = True
      manipulation['detect_silence_threshold'] = 3.0
      manipulation['energy_denoising'] = True
      manipulation['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_1_ch_denoised":
      manipulation['detect_silence'] = True
      manipulation['detect_silence_threshold'] = 3.0
      manipulation['energy_denoising'] = True
      manipulation['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_1_ch_only_sox":
      manipulation['sox_denoising'] = True
      manipulation['sox_denoising_scale'] = 0.15
      manipulation['detect_silence'] = True
      manipulation['detect_silence_threshold'] = 3.0
    elif audio_type == "nao_wav_1_ch_denoised_only_sox":
      manipulation['detect_silence'] = True
      manipulation['detect_silence_threshold'] = 3.0

    return manipulation


  # Performs the speech recognition and returns a list of words
  def performSpeechRecognition(self, audio_file, audio_source, user):
    # Check if path exists
    if os.path.isfile(audio_file) == False:
      return ["Error: Something went wrong with the local audio storage\
              Requested path: " + audio_file]

    # Keep extra audio files that need erasing
    audio_to_be_erased = []

    # If it is an .ogg file (from NAO) recode it into .wav
    next_audio_file = audio_file
    prev_audio_file = next_audio_file

    audio_file_folder = os.path.dirname(audio_file)
    if audio_file_folder[-1] != "/":
      audio_file_folder += "/"

    # Check that the audio_source is legit
    if audio_source not in [\
        "headset", \
        "nao_ogg", \
        "nao_wav_4_ch", \
        "nao_wav_1_ch",\
        "nao_wav_1_ch_denoised", \
        "nao_wav_1_ch_only_sox", \
        "nao_wav_1_ch_denoised_only_sox"\
        ]:
      return ["Error: Audio source unrecognized"]

    # Get processing profile
    profile = self.createProcessingProfile(audio_source)


    transform_req = AudioProcessingTransformAudioSrvRequest()
    transform_req.source_type = audio_source
    transform_req.source_name = prev_audio_file
    transform_req.target_type = 'wav'

    # Check if sox_transform is needed
    if profile['sox_transform'] == True:
      next_audio_file += "_transformed.wav"
      transform_req.target_name = next_audio_file

      trans_response = self.audio_transform_srv( transform_req )

      if trans_response.error != 'success':
          raise RappError( 'Audio transformation error: ' + error )

      #command = "sox " + prev_audio_file + " " + next_audio_file
      #com_res = os.system(command)
      #if com_res != 0:
        #return ["Error: sox malfunctioned"]
      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file
    if profile['sox_channels_and_rate'] == True:
      next_audio_file += "_mono16k.wav"
      transform_req.target_name = next_audio_file
      transform_req.target_channels = 1
      transform_req.target_rate = 16000

      trans_response = self.audio_transform_srv( transform_req )

      if trans_response.error != 'success':
          raise RappError( 'Audio transformation error: ' + error )
      #command = "sox " + prev_audio_file + " -r 16000 -c 1 " + next_audio_file
      #com_res = os.system(command)
      #if com_res != 0:
        #return ["Error: sox malfunctioned"]
      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file
    if profile['sox_denoising'] == True:
      next_audio_file = prev_audio_file + "_denoised.wav"
      den_request = AudioProcessingDenoiseSrvRequest()
      den_request.audio_file = prev_audio_file
      den_request.denoised_audio_file = next_audio_file
      den_request.audio_type = audio_source
      den_request.user = user
      den_request.scale = profile['sox_denoising_scale']
      den_response = self.denoise_service(den_request)
      if den_response.success != "true":
        return ["Error:" + den_response.success]
      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file
    if profile['detect_silence'] == True:
      # Detect silence
      silence_req = AudioProcessingDetectSilenceSrvRequest()
      silence_req.audio_file = prev_audio_file
      silence_req.threshold = profile['detect_silence_threshold']
      silence_res = self.detect_silence_service(silence_req)
      rapp_print("Silence detection results: " + str(silence_res))
      if silence_res.silence == "true":
        return ["Error: No speech detected. RSD = " + str(silence_res.level)]

    tries = 0
    while tries < 2:
        # Perform energy denoising as well
        if profile['energy_denoising'] == True:
          next_audio_file = prev_audio_file + "_energy_denoised.wav"
          dres = self.performEnergyDenoising(next_audio_file, prev_audio_file, \
                  profile['energy_denoising_init_scale'] + tries * 0.125)
          if dres != "true":
            return ["Error:" + dres]
          audio_to_be_erased.append(next_audio_file)
          prev_audio_file = next_audio_file

        new_audio_file = next_audio_file
        words = self.callSphinxJava(new_audio_file)
        if self.sphinxDied == True:
            self.sphinxDied = False
            break

        if len(words) == 0 or (len(words) == 1 and words[0] == ""):
            tries += 1
        else:
            break

    backup_directory = \
        os.path.expanduser("~/rapp_platform_files/rapp_speech_recognition_sphinx4/")\
        + user
    if not os.path.isdir(backup_directory):
      os.makedirs(backup_directory)

    # Keep the original file:
    command = "cp " + audio_file + " " + backup_directory + "/" + \
            audio_file.split("/")[-1]
    com_res = os.system(command)
    if com_res != 0:
      return ["Error: Server cp malfunctioned"]

    for f in audio_to_be_erased:
      clean_file = f.split("/")[-1]
      command = "cp " + f + " " + backup_directory + \
          "/" + clean_file
      os.system(command)
      if com_res != 0:
        return ["Error: Server cp malfunctioned"]

    for f in audio_to_be_erased:
      command = "rm " + f
      os.system(command)
      if com_res != 0:
        return ["Error: Server rm malfunctioned"]


    return words

  def performEnergyDenoising(self, next_audio_file, prev_audio_file, scale):
    energy_denoise_req = AudioProcessingDenoiseSrvRequest()
    energy_denoise_req.audio_file = prev_audio_file
    energy_denoise_req.denoised_audio_file = next_audio_file
    energy_denoise_req.scale = scale
    energy_denoise_res = self.energy_denoise_service(energy_denoise_req)
    return energy_denoise_res.success

  def callSphinxJava(self,new_audio_file):
    self.socket_connection.sendall("start\r\n")
    self.socket_connection.sendall("audioInput#" + new_audio_file + "\r\n")
    start_time = time.time()
    self.readLine()
    words = []
    while(True):
      line = self.readLine()
      if(len(line)>0):
        if(line[0]=="#"):
          stripped_down_line = line[1:-1].split(" ")
          for word in stripped_down_line:
            words.append(word)
        if("stopPython\n" in line):
          break
        if("CatchedException" in line):
            rospy.logerr(line)
            self.respawnSphinx()
            self.sphinxDied = True
            return words

      if (time.time() - start_time > 10):
        words.append("Error: Time out error")
        break
    return words

  def respawnSphinx(self):
    #rospy.logwarn("Respawning sphinx")
    self.p.kill()
    time.sleep(2)
    self.initializeSphinx( self._conf )
    #rospy.logwarn("Respawned sphinx")


