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

        self.user = 'klpanagi'

    def call_face_detection(self, fileUri): 
        # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl['face_detection']
        r = requests.post(url, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        #r = requests.post(url, files=files)

        resp_msg = json.loads(r.text)
        print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg

rappCloud = RappCloud()

fileUri = 'Lenna.png'

words_found = rappCloud.call_face_detection(fileUri)


#print words_found

#for w in words_found:
    #print w

#print  json.dumps(payload)


#r = requests.post(url, data=payload, files=files)
#r = requests.post(url, files=files, data=payload, headers=headers)
#retMsg =  json.loads(r.text)
#for w in retMsg['values']['words']: 
    #print w
