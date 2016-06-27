#!/usr/bin/env python2
# -*- coding: utf-8 -*-

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

import rospy
import httplib
import json
import sys
import os

from pylab import *
from scipy.io import wavfile

from rapp_platform_ros_communications.srv import (
  SpeechToTextSrv,
  SpeechToTextSrvResponse
  )

from rapp_platform_ros_communications.srv import (
  AudioProcessingDenoiseSrv,
  AudioProcessingDenoiseSrvResponse,
  AudioProcessingDenoiseSrvRequest
  )

from rapp_platform_ros_communications.srv import (
  AudioProcessingTransformAudioSrv,
  AudioProcessingTransformAudioSrvResponse,
  AudioProcessingTransformAudioSrvRequest
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from rapp_exceptions import RappError

## @class SpeechToTextGoogle
# Implements calls the Google ASR API
class SpeechToTextGoogle:

  ## Default contructor. Declares the service callback
  def __init__(self):
    # Speech recognition service published
    self.serv_topic = rospy.get_param("rapp_speech_detection_google_detect_speech_topic")
    if(not self.serv_topic):
        rospy.logerror("Speech detection google topic param not found")

    self.serv = rospy.Service(self.serv_topic, \
        SpeechToTextSrv, self.speech_to_text_callback)

  ## @brief The service callback
  # @param req [SpeechToTextSrvRequest] The ROS service request
  def speech_to_text_callback(self, req):

    res = SpeechToTextSrvResponse()

    if req.language == '':
        res.error = 'No language specified'
        return res

    # Getting the results in order to parse them
    try:
        transcripts = self.speech_to_text(\
                req.filename,\
                req.user,\
                req.audio_type,\
                req.language)
        #print transcripts
    except RappError as e:
        res.error = e.value
        return res

    if len(transcripts['result']) == 0:
        return res

    # The alternative results
    alternatives = transcripts['result'][0]['alternative']
    res = SpeechToTextSrvResponse()

    # If alternatives is 0 returns void response
    if len(alternatives) > 0:
      # The first alternative is Google's suggestion
      words = alternatives[0]['transcript'].split(" ")
      for w in words:
        res.words = res.words + [w]
      # Google provides the confidence for the first suggestion
      if 'confidence' in alternatives[0].keys():
        res.confidence.data = alternatives[0]['confidence']
      else:
        res.confidence.data = 0

      for alt in alternatives[1:]:
        sam = StringArrayMsg()
        words = alt['transcript'].split(" ")
        for w in words:
          sam.s = sam.s + [w]
        res.alternatives = res.alternatives + [sam]
    else:
      res.confidence.data = 0
    return res


  ## @brief Performs the call to Google API
  # @param file_path [string] The audio file
  # @param user [string] The username
  # @param audio_type [string] Used to check if denoising is needed. Can be one of headset, nao_ogg, nao_wav_1_ch, nao_wav_4_ch
  # @param language [string] The language in which the ASR will be performed
  # @return The transcript from Google
  def speech_to_text(self, file_path, user, audio_file_type, language):

    # Check if file exists
    if not os.path.isfile(file_path):
        raise RappError("Error: file " + file_path + ' not found')

    # Check if file is flac. If not convert it
    new_audio = file_path

    audio_trans_topic = rospy.get_param("rapp_audio_processing_transform_audio_topic")
    audio_transform_srv = rospy.ServiceProxy( audio_trans_topic, AudioProcessingTransformAudioSrv )

    cleanup = []

    transform_req = AudioProcessingTransformAudioSrvRequest()
    transform_req.source_type = audio_file_type
    transform_req.source_name = new_audio
    transform_req.target_type = 'wav'
    new_audio += '.wav'
    transform_req.target_name = new_audio
    transform_req.target_channels = 1
    transform_req.target_rate = 16000

    trans_response = audio_transform_srv( transform_req )

    if trans_response.error != 'success':
        raise RappError( trans_response.error )
    cleanup.append(new_audio)

    # Denoise if necessary
    prev_audio_file = new_audio
    next_audio_file = prev_audio_file
    if audio_file_type in ['nao_ogg', 'nao_wav_1_ch', 'nao_wav_4_ch']:
        denoise_topic = rospy.get_param("rapp_audio_processing_denoise_topic")
        energy_denoise_topic = \
            rospy.get_param("rapp_audio_processing_energy_denoise_topic")
        denoise_service = rospy.ServiceProxy(\
            denoise_topic, AudioProcessingDenoiseSrv)
        energy_denoise_service = rospy.ServiceProxy(\
            energy_denoise_topic, AudioProcessingDenoiseSrv)

        manipulation = {}
        manipulation['sox_transform'] = False
        manipulation['sox_denoising'] = False
        manipulation['sox_channels_and_rate'] = False
        if audio_file_type == "headset":
            pass
        elif audio_file_type == "nao_ogg":
            manipulation['sox_transform'] = True
            manipulation['sox_denoising'] = True
            manipulation['sox_denoising_scale'] = 0.15
        elif audio_file_type == "nao_wav_4_ch":
            manipulation['sox_channels_and_rate'] = True
            manipulation['sox_denoising'] = True
            manipulation['sox_denoising_scale'] = 0.15
        elif audio_file_type == "nao_wav_1_ch":
            manipulation['sox_denoising'] = True
            manipulation['sox_denoising_scale'] = 0.15
            manipulation['detect_silence'] = True
            manipulation['detect_silence_threshold'] = 0.25

        # Check if sox_transform is needed
        if manipulation['sox_transform'] == True:
            next_audio_file += "_transformed.wav"
            command = "sox " + prev_audio_file + " " + next_audio_file
            com_res = os.system(command)
            if com_res != 0:
                raise RappError("Error: sox malfunctioned")
            cleanup.append(next_audio_file)
            prev_audio_file = next_audio_file
        if manipulation['sox_channels_and_rate'] == True:
            next_audio_file += "_mono16k.wav"
            command = "sox " + prev_audio_file + " -r 16000 -c 1 " + next_audio_file
            com_res = os.system(command)
            if com_res != 0:
                raise RappError("Error: sox malfunctioned")
            cleanup.append(next_audio_file)
            prev_audio_file = next_audio_file
        if manipulation['sox_denoising'] == True:
            next_audio_file = prev_audio_file + "_denoised.wav"
            den_request = AudioProcessingDenoiseSrvRequest()
            den_request.audio_file = prev_audio_file
            den_request.denoised_audio_file = next_audio_file
            den_request.audio_type = audio_file_type
            den_request.user = user
            den_request.scale = manipulation['sox_denoising_scale']
            den_response = denoise_service(den_request)
            if den_response.success != "true":
                raise RappError("Error:" + den_response.success)
            cleanup.append(next_audio_file)
            prev_audio_file = next_audio_file

            # must implement a fallback function to clear redundant files

    # Transform to flac
    transform_req = AudioProcessingTransformAudioSrvRequest()
    transform_req.source_type = 'headset'
    transform_req.source_name = new_audio
    transform_req.target_type = 'flac'
    newer_audio = new_audio + '.flac'
    transform_req.target_name = newer_audio
    transform_req.target_channels = 1
    transform_req.target_rate = 16000

    trans_response = audio_transform_srv( transform_req )
    cleanup.append(newer_audio)

    if trans_response.error != 'success':
        raise RappError( trans_response.error )


    # Open the file
    with open(newer_audio, "r") as f:
      speech = f.read()
    url = "www.google.com"

    # Fix language
    if language == 'en':
        language = "en-US"
    elif language == 'gr':
        language = 'el'

    #NOTE - Thats a general usage key. They may disable it in the future.
    key = "AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw"
    path = "/speech-api/v2/recognize?lang=" + language + "&key=" + key
    headers = { "Content-type": "audio/x-flac; rate=22050" };
    params = {"xjerr": "1", "client": "chromium"}
    conn = httplib.HTTPSConnection(url)
    conn.request("POST", path, speech, headers)
    response = conn.getresponse()
    data = response.read()
    initial_data = data
    # Google returns one empty result for some reason here. Removing it..
    index = data.find("}")
    data = data[index + 1:]
    if data == '\n':
        # Returned nothing.. something went wrong
        data = initial_data
    jsdata = json.loads(data)

    # Remove the flac if needed
    for f in cleanup:
        command = 'rm -f ' + f
        if os.system(command):
            raise RappError("Error: Removal of temporary file malfunctioned")
    return jsdata

## The main function. Creates a SpeechToTextGoogle object
if __name__ == "__main__":
  rospy.init_node('speech_to_text_ros_node')
  speech_to_text_node = SpeechToTextGoogle()
  rospy.spin()


