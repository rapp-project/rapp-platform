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

## @class Sphinx4Wrapper
# @brief Contains the Sphinx subprocess and is responsible for configuring Sphinx and performing the recognition request.
#
# Initializes a Sphinx.java subprocess and creates an IPC using sockets.
# It is responsible for interacting with Sphinx via the socket to send
# configuration params/instructions and initialize a recognition procedure.
class Sphinx4Wrapper():

  ## Constructor
  # Initiates service clients
  def __init__(self):
    ## Contains global Sphinx parameters
    #
    # (see global_parameters.GlobalParams)
    self._globalParams = GlobalParams()

    ## Sphinx configuration
    self._conf = ''
    ## Sphinx status flag
    self._sphinxDied = False

    ## The IPC socket
    self._sphinx_socket = None
    ## The IPC socket port
    self._sphinx_socket_PORT = None

    ## The Sphinx subprocess
    self._sphinxSubprocess = None

    # Denoise service topic name
    denoise_topic = rospy.get_param("rapp_audio_processing_denoise_topic")
    # Energy denoise service topic name
    energy_denoise_topic = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")
    # Detect silence service topic name
    detect_silence_topic = \
        rospy.get_param("rapp_audio_processing_detect_silence_topic")
    # Transform audio service topic name
    audio_trans_topic = \
        rospy.get_param("rapp_audio_processing_transform_audio_topic")

    if(not denoise_topic):
      rospy.logerror("Audio processing denoise topic not found")
    if(not energy_denoise_topic):
      rospy.logerror("Audio processing energy denoise topic not found")
    if(not detect_silence_topic):
      rospy.logerror("Audio processing detect silence topic not found")
    if(not audio_trans_topic):
      rospy.logerror("Audio processing transform audio topic not found")

    ## @brief Denoise service client
    #
    # rapp_audio_processing.rapp_audio_processing.AudioProcessing::denoise
    self._denoise_service = rospy.ServiceProxy(\
              denoise_topic, AudioProcessingDenoiseSrv)

    ## @brief Energy denoise service client
    #
    # rapp_audio_processing.rapp_audio_processing.AudioProcessing::energy_denoise
    self._energy_denoise_service = rospy.ServiceProxy(\
              energy_denoise_topic, AudioProcessingDenoiseSrv)

    ## @brief Detect silence service client
    #
    # rapp_audio_processing.rapp_audio_processing.AudioProcessing::detect_silence
    self._detect_silence_service = rospy.ServiceProxy(\
              detect_silence_topic, AudioProcessingDetectSilenceSrv)

    ## @brief Transform audio service client
    #
    # rapp_audio_processing.rapp_audio_processing.AudioProcessing::transform_audio
    self._audio_transform_srv = rospy.ServiceProxy( \
        audio_trans_topic, AudioProcessingTransformAudioSrv )

    ## Contains the absolute path for the Sphinx jar file
    self._jar_path = ".:" + self._globalParams._sphinx_jar_files_url + \
        "/" + self._globalParams._sphinx_jar_file + ":" \
            + self._globalParams._sphinx_package_url + "/src"

    self._initializeSphinxProcess()

  ## Helper function for getting input from IPC with Sphinx subprocess
  #
  # @return line [string] A buffer read from socket
  def _readLine(self):
    line = self.socket_connection.recv(1024)
    if self._globalParams._allow_sphinx_output == True:
      rapp_print( line )
    return line

  ## Perform Sphinx4 initialization
  # Initiates Sphinx subprocess, sets up socket IPC and configures Sphinx subprocess
  #
  # @param conf [dictionary] Contains the configuration parameters
  def _initializeSphinxProcess(self, conf = None):

    rapp_print('Initializing Sphinx subprocess')
    rapp_print(self._jar_path)

    rapp_print('Setting up socket IPC')
    self._createSocket()

    rapp_print('Forking subprocess')
    if self._globalParams._allow_sphinx_output == True:
      self._sphinxSubprocess = subprocess.Popen( \
          ["java", "-cp", self._jar_path, "Sphinx4", \
             str(self._sphinx_socket_PORT)] )
    else:
      try:
        from subprocess import DEVNULL
      except ImportError:
        DEVNULL = open(os.devnull, 'wb')

      self._sphinxSubprocess = subprocess.Popen( \
          ["java", "-cp", self._jar_path, "Sphinx4", \
              str(self._sphinx_socket_PORT)], \
            stdout = DEVNULL, stderr = DEVNULL )

    rapp_print('Awaiting socket connection')
    self.socket_connection, addr = self._sphinx_socket.accept()

    if conf != None:
      self.configureSphinx( conf )

  ## Creates socket IPC between self and Sphinx subprocess
  # Creates the socket server with a system provided port, which is pass as an
  # argument to the created subprocess.
  def _createSocket(self):
    HOST = self._globalParams._socket_host
    self._sphinx_socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM ) # Create Unix Socket
    self._sphinx_socket.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self._sphinx_socket.bind( (HOST, 0) )
    self._sphinx_socket_PORT = self._sphinx_socket.getsockname()[1]
    self._sphinx_socket.listen( 1 )
    rapp_print('Socket created. PORT: ' + str(self._sphinx_socket_PORT))

  ## Perform Sphinx4 configuration
  #
  # @param conf [dictionary] Contains the configuration parameters
  def configureSphinx(self, conf):
    self._conf = conf
    self.socket_connection.sendall("configurationPath#" + conf['configuration_path'] + '\r\n')
    self._readLine()
    self.socket_connection.sendall("acousticModel#" + conf['acoustic_model'] + '\r\n')
    self._readLine()
    self.socket_connection.sendall("grammarName#" + conf['grammar_name'] + "#" + \
            conf['grammar_folder'] + '\r\n')
    self._readLine()
    self.socket_connection.sendall("dictionary#" + conf['dictionary'] + '\r\n')
    self._readLine()
    self.socket_connection.sendall("languageModel#" + conf['language_model'] + '\r\n')
    self._readLine()
    if(conf['grammar_disabled']):
      self.socket_connection.sendall("disableGrammar#\r\n")
    else:
      self.socket_connection.sendall("enableGrammar#\r\n")
    self._readLine()
    self.socket_connection.sendall("forceConfiguration#\r\n")
    self._readLine()

  ## Creates audio profile based on the audio type for processing purposes.
  # Defines a set of audio processing procedures (i.e. denoising) to be
  # performed on the audio file and the parameters of the procedures.
  # Aims to improve to audio file quality to improve speech recognition results.
  #
  # @param audio_type [string] The audio type
  #
  # @return processingProfile [dictionary] The profile attributes
  def _createProcessingProfile(self, audio_type):
    processingProfile = {}
    processingProfile['sox_transform'] = False
    processingProfile['sox_channels_and_rate'] = False
    processingProfile['sox_denoising'] = False
    processingProfile['sox_denoising_scale'] = 0.0
    processingProfile['detect_silence'] = False
    processingProfile['detect_silence_threshold'] = 0.0
    processingProfile['energy_denoising'] = False
    processingProfile['energy_denoising_init_scale'] = 0.0

    if audio_type == "headset":
      pass
    elif audio_type == "nao_ogg":
      processingProfile['sox_transform'] = True
      processingProfile['sox_denoising'] = True
      processingProfile['sox_denoising_scale'] = 0.15
      processingProfile['detect_silence'] = True
      processingProfile['detect_silence_threshold'] = 3.0
      processingProfile['energy_denoising'] = True
      processingProfile['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_4_ch":
      processingProfile['sox_channels_and_rate'] = True
      processingProfile['sox_denoising'] = True
      processingProfile['sox_denoising_scale'] = 0.15
      processingProfile['detect_silence'] = True
      processingProfile['detect_silence_threshold'] = 3.0
      processingProfile['energy_denoising'] = True
      processingProfile['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_1_ch":
      processingProfile['sox_denoising'] = True
      processingProfile['sox_denoising_scale'] = 0.15
      processingProfile['detect_silence'] = True
      processingProfile['detect_silence_threshold'] = 3.0
      processingProfile['energy_denoising'] = True
      processingProfile['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_1_ch_denoised":
      processingProfile['detect_silence'] = True
      processingProfile['detect_silence_threshold'] = 3.0
      processingProfile['energy_denoising'] = True
      processingProfile['energy_denoising_init_scale'] = 0.125
    elif audio_type == "nao_wav_1_ch_only_sox":
      processingProfile['sox_denoising'] = True
      processingProfile['sox_denoising_scale'] = 0.15
      processingProfile['detect_silence'] = True
      processingProfile['detect_silence_threshold'] = 3.0
    elif audio_type == "nao_wav_1_ch_denoised_only_sox":
      processingProfile['detect_silence'] = True
      processingProfile['detect_silence_threshold'] = 3.0

    return processingProfile


  ## Performs the speech recognition and returns a list of words
  #
  # @param audio_file [string] The audio file's name
  # @param audio_type [string] The audio file's type
  #
  # @returns words [list::string] The result words
  # @exception RappError Audio transformation error
  def performSpeechRecognition(self, audio_file, audio_type, user):
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

    # Check that the audio_type is legit
    if audio_type not in [\
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
    profile = self._createProcessingProfile(audio_type)

    transform_req = AudioProcessingTransformAudioSrvRequest()
    transform_req.source_type = audio_type
    transform_req.source_name = prev_audio_file
    transform_req.target_type = 'wav'

    # Check if sox_transform is needed
    if profile['sox_transform'] == True:
      next_audio_file += "_transformed.wav"
      transform_req.target_name = next_audio_file

      trans_response = self._audio_transform_srv( transform_req )

      if trans_response.error != 'success':
          return [ 'Audio transformation error: ' + error ]
          #raise RappError( 'Audio transformation error: ' + error )

      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file
    if profile['sox_channels_and_rate'] == True:
      next_audio_file += "_mono16k.wav"
      transform_req.target_name = next_audio_file
      transform_req.target_channels = 1
      transform_req.target_rate = 16000

      trans_response = self._audio_transform_srv( transform_req )

      if trans_response.error != 'success':
          return [ 'Audio transformation error: ' + error ]
          #raise RappError( 'Audio transformation error: ' + error )
      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file
    if profile['sox_denoising'] == True:
      next_audio_file = prev_audio_file + "_denoised.wav"
      den_request = AudioProcessingDenoiseSrvRequest()
      den_request.audio_file = prev_audio_file
      den_request.denoised_audio_file = next_audio_file
      den_request.audio_type = audio_type
      den_request.user = user
      den_request.scale = profile['sox_denoising_scale']
      den_response = self._denoise_service(den_request)
      if den_response.success != "true":
        return ["Error:" + den_response.success]
      audio_to_be_erased.append(next_audio_file)
      prev_audio_file = next_audio_file
    if profile['detect_silence'] == True:
      # Detect silence
      silence_req = AudioProcessingDetectSilenceSrvRequest()
      silence_req.audio_file = prev_audio_file
      silence_req.threshold = profile['detect_silence_threshold']
      silence_res = self._detect_silence_service(silence_req)
      rapp_print("Silence detection results: " + str(silence_res))
      if silence_res.silence == "true":
        return ["Error: No speech detected. RSD = " + str(silence_res.level)]

    tries = 0
    while tries < 2:
        # Perform energy denoising as well
        if profile['energy_denoising'] == True:
          next_audio_file = prev_audio_file + "_energy_denoised.wav"
          dres = self._performEnergyDenoising(next_audio_file, prev_audio_file, \
                  profile['energy_denoising_init_scale'] + tries * 0.125)
          if dres != "true":
            return ["Error:" + dres]
          audio_to_be_erased.append(next_audio_file)
          prev_audio_file = next_audio_file

        new_audio_file = next_audio_file
        words = self._callSphinxJava(new_audio_file)
        if self._sphinxDied == True:
            self._sphinxDied = False
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

  ## Perform energy denoise.
  # Calls energy denoise service
  # rapp_audio_processing.rapp_audio_processing.AudioProcessing::energy_denoise
  # (see also rapp_audio_processing.rapp_energy_denoise.EnergyDenoise::energyDenoise)
  #
  # @param audio_file [string] The audio file path
  # @param scale      [float] The scale parameter for the denoising procedure
  #
  # @returns file_path [string] THe path of the denoised file
  def _performEnergyDenoising(self, next_audio_file, audio_file, scale):
    energy_denoise_req = AudioProcessingDenoiseSrvRequest()
    energy_denoise_req.audio_file = audio_file
    energy_denoise_req.denoised_audio_file = next_audio_file
    energy_denoise_req.scale = scale
    energy_denoise_res = self._energy_denoise_service(energy_denoise_req)
    return energy_denoise_res.success

  ## Communicate with Sphinx subprocess to initiate recognition and fetch results.
  #
  # @param audio_file [string] The audio file path
  #
  # @return words [list::string] The Sphinx result
  def _callSphinxJava(self, audio_file):
    self.socket_connection.sendall("start\r\n")
    self.socket_connection.sendall("audioInput#" + audio_file + "\r\n")
    start_time = time.time()
    words = []
    while(True):
      line = self._readLine()
      self.socket_connection.sendall('Read line\r\n')
      if(len(line)>0):
        if(line[0]=="#"):
          stripped_down_line = line[1:-1].split(" ")
          for word in stripped_down_line:
            words.append(word)
        if("stopPython\n" in line):
          break
        if("CatchedException" in line):
            rospy.logerr(line)
            self._respawnSphinx()
            self._sphinxDied = True
            return words

      if (time.time() - start_time > 10):
        words.append("Error: Time out error")
        break
    return words

  ## Respawns Sphinx subprocess, if it terminates abruptly
  def _respawnSphinx(self):
    #rospy.logwarn("Respawning sphinx")
    self._sphinxSubprocess.kill()
    time.sleep(2)
    self._initializeSphinxProcess( self._conf )
    #rospy.logwarn("Respawned sphinx")


