#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
from requests.auth import HTTPBasicAuth
import json


class RappCloud:

    def __init__(self):
        self.serviceUrl = {}
        self.serviceUrl['speech_detection'] = 'http://155.207.19.13:9001/hop/speech_detection_sphinx4'
        self.user = 'klpanagi'

    def call_speech_detection(self, language, audio_source, words, sentences, grammar, fileUri): 

        # -- Craft the data payload for the post request
        payload = {'language':language, 'audio_source':audio_source, \
                'words':json.dumps(words), 'sentences':json.dumps(sentences), \
                'grammar':json.dumps(grammar), 'user': self.user}

        # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl['speech_detection']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))

        resp_msg = json.loads(r.text)
        print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg

rappCloud = RappCloud()

# --- Define the payload
language = 'gr'
audio_source = 'nao_wav_1_ch'
words = []
words.append(u'ναι')
words.append(u'οχι')
sentences = [u'ναι',u'οχι']
grammar = []
fileUri = 'nai-oxi-test.wav'
resp = rappCloud.call_speech_detection(language, audio_source, \
        words, sentences, grammar, fileUri)


