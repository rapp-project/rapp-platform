#!/usr/bin/env python2
# -*- coding: utf-8 -*-

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

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

from rapp_platform_ros_communications.srv import (
  fetchDataSrv,
  fetchDataSrvRequest
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from rapp_exceptions import RappError

class SpeechToTextGoogle:

  def __init__(self):
    # Speech recognition service published
    self.serv_topic = rospy.get_param("rapp_speech_detection_google_detect_speech_topic")
    if(not self.serv_topic):
        rospy.logerror("Speech detection google topic param not found")

    self.threads = rospy.get_param("rapp_speech_detection_google_threads")
    if not self.threads:
        rospy.logerror("Speech detection google threads param not found")
        self.threads = 0

    self.serv = rospy.Service(self.serv_topic, \
        SpeechToTextSrv, self.speech_to_text_callback)
    for i in range(0, self.threads):
        rospy.Service(self.serv_topic + '_' + str(i), \
            SpeechToTextSrv, self.speech_to_text_callback)

  # The service callback
  def speech_to_text_callback(self, req):

    res = SpeechToTextSrvResponse()

    # Getting the results in order to parse them
    try:
        transcripts = self.speech_to_text(\
                req.filename,\
                req.user,\
                req.audio_type,\
                req.language)
        print transcripts
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


  #NOTE The audio file should be flac to work.
  def speech_to_text(self, file_path, user, audio_file_type, language):

    # Check the user
    serv_db_topic = rospy.get_param("rapp_mysql_wrapper_user_fetch_data_topic")
    authentication_service = rospy.ServiceProxy(serv_db_topic, fetchDataSrv)
    req_db = fetchDataSrv()
    req_db.req_cols=["username"]
    entry1=["username", user]
    req_db.where_data=[StringArrayMsg(s=entry1)]

    resp = authentication_service(req_db.req_cols, req_db.where_data)
    if resp.success.data != True or len(resp.res_data) == 0:
      raise RappError("Non authenticated user")

    # Check if file exists
    if not os.path.isfile(file_path):
        raise RappError("Error: file " + file_path + ' not found')

    # Check if file is flac. If not convert it
    new_audio = file_path


    audio_trans_topic = rospy.get_param("rapp_audio_processing_transform_audio_topic")
    audio_transform_srv = rospy.ServiceProxy( audio_trans_topic, AudioProcessingTransformAudioSrv )


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
        raise RappError( error )

    # Transform it to wav, 1 channel
    cleanup = []
    #if audio_file_type == 'nao_ogg':
        #if ".ogg" not in new_audio:
            #raise RappError("Error: ogg type selected but file is of another type")
        #new_audio += ".wav"
        #com_res = os.system("sox " + file_path + " " + new_audio)
        #if com_res != 0:
            #raise RappError("Error: Server sox malfunctioned")
        #cleanup.append(new_audio)

    #elif audio_file_type == "nao_wav_1_ch" or audio_file_type == 'headset':
        #if ".wav" not in new_audio:
            #raise RappError("Error: wav type 1 channel selected but file is of another type")
        #samp_freq, signal = wavfile.read(new_audio)
        #if len(signal.shape) != 1:
            #raise RappError("Error: wav 1 ch declared but the audio file has " +\
                #str(signal.shape[1]) + ' channels')

    #elif audio_file_type == "nao_wav_4_ch":
        #if ".wav" not in new_audio:
            #raise RappError("Error: wav type 4 channels selected but file is of another type")
        #samp_freq, signal = wavfile.read(new_audio)
        #if len(signal.shape) != 2 or signal.shape[1] != 4:
            #raise RappError("Error: wav 4 ch declared but the audio file has not 4 channels")
        #new_audio += "_1ch.wav"
        #com_res = os.system("sox " + file_path + " -c 1 -r 16000 " + \
            #new_audio)
        #if com_res != 0:
            #raise RappError("Error: Server sox malfunctioned")
        #cleanup.append(new_audio)

    #else:
        #msg = ''
        #msg = "Non valid noise audio type"
        #for f in cleanup:
            #os.system('rm ' + f)
        #raise RappError(msg)

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
    newer_audio = prev_audio_file + '.flac'
    command = 'flac -f --channels=1 --sample-rate=16000 '\
            + new_audio + ' -o ' + newer_audio
    cleanup.append(newer_audio)
    if os.system(command):
        raise RappError("Error: flac command malfunctioned. File path was"\
                + new_audio)

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

if __name__ == "__main__":
  rospy.init_node('speech_to_text_ros_node')
  speech_to_text_node = SpeechToTextGoogle()
  rospy.spin()


