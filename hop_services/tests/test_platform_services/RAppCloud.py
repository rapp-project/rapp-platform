#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
from requests.auth import HTTPBasicAuth
import json


class RappCloud:

    def __init__(self):
        self.serviceUrl = {}
        self.serviceUrl['speech_detection'] = \
            'http://155.207.19.37:9001/hop/speech_detection_sphinx4'
        self.serviceUrl['set_denoise_profile'] = \
            'http://155.207.19.37:9001/hop/set_denoise_profile'
        self.serviceUrl['face_detection'] = 'http://155.207.19.37:9001/hop/face_detection'
        self.serviceUrl['qr_detection'] = 'http://155.207.19.37:9001/hop/qr_detection'
        self.serviceUrl['ontology_subclassesOf'] = \
            'http://155.207.19.37:9001/hop/ontology_subclassesOf'

        self.user = 'klpanagi'


    #===========================================================================================
    def call_speech_detection(self, language, audio_source, words, sentences, grammar, fileUri): 

        # -- Craft the data payload for the post request
        payload = {'language':language, 'audio_source':audio_source, \
                'words':json.dumps(words), 'sentences':json.dumps(sentences), \
                'grammar':json.dumps(grammar), 'user': self.user}

        # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl['speech_detection']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', \
            'rappdev'))

        resp_msg = json.loads(r.text)
        print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg
    #===========================================================================================


    #===========================================================================================
    def call_set_denoise_profile(self, noise_audio_fileUri, audio_file_type, user):
        # -- Craft the data payload for the post request
        payload = {'user': user, 'audio_file_type': audio_file_type}

        # -- Files to be added into to poset request
        files = {'noise_audio_fileUri': open(noise_audio_fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl['set_denoise_profile']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', \
            'rappdev'))

        resp_msg = json.loads(r.text)
        print  resp_msg
        return resp_msg 
    #===========================================================================================


    #===========================================================================================
    def call_qr_detection(self, fileUri): 
        # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl['qr_detection']
        r = requests.post(url, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        #r = requests.post(url, files=files)

        resp_msg = json.loads(r.text)
        print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg
    #===========================================================================================


    #===========================================================================================
    def call_ontology_subclassesOf(self, query):
        payload = {'queryStr': query}
        url = self.serviceUrl['ontology_subclassesOf']
        r = request.post(url, data=payload, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        resp_msg = json.loads(r.text)
        return resp_msg
    #===========================================================================================

rappCloud = RappCloud()

