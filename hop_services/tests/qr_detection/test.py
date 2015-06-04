#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
from requests.auth import HTTPBasicAuth
import json


class RappCloud:

    def __init__(self):
        self.serviceUrl = {}
        self.serviceUrl['speech_detection'] = 'http://155.207.19.37:9001/hop/speech_detection_sphinx4'
        self.serviceUrl['face_detection'] = 'http://155.207.19.37:9001/hop/face_detection'
        self.serviceUrl['qr_detection'] = 'http://155.207.19.37:9001/hop/qr_detection'

        self.user = 'klpanagi'

    def call_face_detection(self, fileUri): 

        # -- Craft the data payload for the post request
        #payload = {'language':language, 'audio_source':audio_source, \
                #'words':json.dumps(words), 'sentences':json.dumps(sentences), \
                #'grammar':json.dumps(grammar), 'user': self.user}

        # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl['face_detection']
        r = requests.post(url, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        #r = requests.post(url, files=files)

        resp_msg = json.loads(r.text)
        print '\033[1;33m' + resp_msg + '\033[0m'
        return resp_msg


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


rappCloud = RappCloud()

fileUri = 'qr_code_rapp.jpg'

resp = rappCloud.call_qr_detection(fileUri)

