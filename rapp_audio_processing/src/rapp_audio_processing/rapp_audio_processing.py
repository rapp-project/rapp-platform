#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Authors: Manos Tsardoulias
# contact: etsardou@iti.gr

import rospy
import sys
import time
import os
from pylab import *
from scipy.io import wavfile

from rapp_platform_ros_communications.srv import (  
  AudioProcessingDenoiseSrv, 
  AudioProcessingDenoiseSrvResponse,
  
  AudioProcessingSetNoiseProfileSrv,
  AudioProcessingSetNoiseProfileSrvResponse,

  AudioProcessingDetectSilenceSrv,
  AudioProcessingDetectSilenceSrvResponse
  )

from rapp_platform_ros_communications.srv import (
  fetchDataSrv,
  fetchDataSrvRequest
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

from std_msgs.msg import ( 
  String 
  ) 

from rapp_detect_silence import DetectSilence
from rapp_energy_denoise import EnergyDenoise


class AudioProcessing:
 
  # Constructor performing initializations
  def __init__(self):    

    self.detect_silence_module = DetectSilence()
    self.energy_denoise_module = EnergyDenoise()

    self.set_noise_profile_topic = rospy.get_param(\
            "rapp_audio_processing_set_noise_profile_topic")
    self.denoise_topic = \
        rospy.get_param("rapp_audio_processing_denoise_topic")
    self.energy_denoise_topic = \
        rospy.get_param("rapp_audio_processing_energy_denoise_topic")
    self.detect_silence_topic = \
        rospy.get_param("rapp_audio_processing_detect_silence_topic")

    if(not self.set_noise_profile_topic):
      rospy.logerror("Audio processing noise profiling topic param not found")
    if(not self.denoise_topic):
      rospy.logerror("Audio processing denoise topic param not found")
    if(not self.energy_denoise_topic):
      rospy.logerror("Audio processing energy denoise topic param not found")
    if(not self.detect_silence_topic):
      rospy.logerror("Audio processing detect silence topic param not found")

    self.energy_denoising_debug = False
    self.energy_denoising_debug = \
        rospy.get_param("rapp_audio_processing_energy_denoising_debug")
    if not self.energy_denoising_debug:
      self.energy_denoising_debug = False
    else:
      self.energy_denoising_debug = True

    self.set_noise_profile_service = rospy.Service(self.set_noise_profile_topic, \
        AudioProcessingSetNoiseProfileSrv, self.setNoiseProfile)
    self.denoise_service = rospy.Service( \
        self.denoise_topic, AudioProcessingDenoiseSrv, \
        self.denoise)
    self.energy_denoise_service = rospy.Service( \
        self.energy_denoise_topic, AudioProcessingDenoiseSrv, \
        self.energy_denoise)
    self.detect_silence_service = rospy.Service( \
        self.detect_silence_topic, AudioProcessingDetectSilenceSrv, \
        self.detect_silence)

    self.serv_db_topic = rospy.get_param("rapp_mysql_wrapper_user_fetch_data_topic")
    self.authentication_service = rospy.ServiceProxy(\
        self.serv_db_topic, fetchDataSrv)
 
  # Service callback for setting noise profile 
  def setNoiseProfile(self, req):
    res = AudioProcessingSetNoiseProfileSrvResponse()

    cleanup = []
    #-------------------------Check with database-------------------------#
    req_db = fetchDataSrv()
    req_db.req_cols=["username"]
    entry1=["username", req.user]
    req_db.where_data=[StringArrayMsg(s=entry1)]

    resp = self.authentication_service(req_db.req_cols, req_db.where_data)
    print resp
    if resp.success.data != True or len(resp.res_data) == 0: 
      res.success = "Non authenticated user"
      return total_res

    directory = "/tmp/rapp_platform_files/audio_processing/" + req.user
    if not os.path.isdir(directory):
      os.makedirs(directory)
      com_res = os.system("chmod 777 " + directory)
      if com_res != 0:
        res.success = "Error: Server chmod malfunctioned"
        return res

    directory += "/noise_profile/"
    if not os.path.isdir(directory):
      os.makedirs(directory)
      com_res = os.system("chmod 777 " + directory)
      if com_res != 0:
        res.success = "Error: Server chmod malfunctioned"
        return res

    noise_profile_file = directory
    new_audio = req.noise_audio_file

    # Making audio compatible to sphinx4
    if req.audio_file_type == 'nao_ogg':
      new_audio += ".wav"
      com_res = os.system("sox " + req.noise_audio_file + " " + new_audio)
      if com_res != 0:
        res.success = "Error: Server sox malfunctioned"
        return res
      cleanup.append(new_audio)
    elif req.audio_file_type == "nao_wav_1_ch":
      pass
    elif req.audio_file_type == "nao_wav_4_ch":
      new_audio += "_1ch.wav"
      com_res = os.system("sox " + req.noise_audio_file + " -c 1 -r 16000 " + \
          new_audio)
      if com_res != 0:
        res.success = "Error: Server sox malfunctioned"
        return res

      cleanup.append(new_audio)
    else:
      res.success = "Non valid noise audio type"
      status = self.cleanup(cleanup)
      if status != True:
        res.success += " " + status
      return total_res

    noise_profile_uri = directory + "/noise_profile_" + req.audio_file_type
    # Extract noise_profile
    com_res = os.system("sox " + new_audio + " -t null /dev/null trim 0.5 2.5 noiseprof "\
            + noise_profile_uri)
    if com_res != 0:
      res.success = "Error: Server sox malfunctioned"
      return res

    com_res = os.system("chmod 777 " + noise_profile_uri)
    if com_res != 0:
      res.success = "Error: Server chmod malfunctioned"
      return res

    status = self.cleanup(cleanup)
    if status != True:
      res.success = status
    else:
      res.success = "true"
    return res

  # Service callback for handling denoising
  def denoise(self, req):     
    res = AudioProcessingDenoiseSrvResponse()
    directory = "/tmp/rapp_platform_files/audio_processing/" + req.user
    noise_profile = directory + "/noise_profile/noise_profile_" + req.audio_type
    if not os.path.isfile(noise_profile):
      res.success = "No noise profile for the " + req.audio_type + " type exists"
      return res
    
    command = "sox " + req.audio_file + " " + req.denoised_audio_file +\
            " noisered " + noise_profile + " " + str(req.scale)
    com_res = os.system(command)
    if com_res != 0:
      res.success = "System sox malfunctioned"
    else:
      res.success = "true"
    return res

  # Service callback for detecting silence
  def detect_silence(self, req):     
    res = AudioProcessingDetectSilenceSrvResponse()
    [res.level, res.silence] = self.detect_silence_module.detectSilence(\
            req.audio_file, req.threshold)
    if res.silence == True:
        res.silence = "true"
    else:
        res.silence = "false"
    return res

  # Service callback for energy denoising
  def energy_denoise(self, req):     
    res = AudioProcessingDenoiseSrvResponse()
    output = self.energy_denoise_module.energyDenoise(\
          req.audio_file, req.scale, req.denoised_audio_file,\
          self.energy_denoising_debug)
    if output == True:
        res.success = "true"
    else:
        res.success = "false"
    return res


  # Cleanup method
  def cleanup(self, clean):
    for f in clean:
      command = "rm " + f
      com_res = os.system(command)
      if com_res != 0:
        return "Error: Server rm malfunctioned"
    return True


# Main function
if __name__ == "__main__": 
  rospy.init_node('AudioProcessing')
  AudioProcessingNode = AudioProcessing()
  rospy.spin()
