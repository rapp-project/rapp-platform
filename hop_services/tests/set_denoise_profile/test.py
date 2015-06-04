#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
from requests.auth import HTTPBasicAuth
import json


class RappCloud:

    def __init__(self):
        self.serviceUrl = {}
        self.serviceUrl['speech_detection'] = 'http://155.207.19.37:9001/hop/speech_detection_sphinx4'
        self.serviceUrl['set_denoise_profile'] = 'http://155.207.19.37:9001/hop/set_denoise_profile/'
        self.user = 'klpanagi'

    def call_set_denoise_profile(self, noise_audio_fileUri, audio_file_type, user):
    # -- Craft the data payload for the post request
        payload = {'user': user, 'audio_file_type': audio_file_type}

        # -- Files to be added into to poset request
        files = {'noise_audio_fileUri': open(noise_audio_fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl['set_denoise_profile']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        resp_msg = json.loads(r.text)
        print  resp_msg
        return resp_msg 

   
rappCloud = RappCloud()

audio_source = 'nao_wav_1_ch'
fileUri = 'nai-oxi-test.wav'
user = 'klpanagi'
resp = rappCloud.call_set_denoise_profile(fileUri, audio_source, user)


