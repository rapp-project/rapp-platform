#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
from requests.auth import HTTPBasicAuth
import json
from pprint import pprint
import os

## --  Set and hold RappCloud directory path -- ##
__path__ = os.path.dirname(__file__)
## -------------------------------------------- ##


class RappCloud:

    def __init__(self):
        # --- Load Rapp Platform parameters --- #
        params = self.load_parameters()
        self.platform_params_ = params['platform']
        # ------------------------------------- #

        self.platformIP_ = self.platform_params_['host_ip']
        self.services_ = [] 
        self.serviceUrl_ = {}
        self.servicePort_ = self.platform_params_['services']['port']

        for service in self.platform_params_['services']['name']:
            self.services_.append(service)
            #print "Service:  [%s]" % 'http://' + self.platformIP_ + ':' + str(self.servicePort_) + '/hop/' + \
                    #service
            self.serviceUrl_[service] = 'http://' + self.platformIP_ + ':' + str(self.servicePort_) + '/hop/' + \
                    service
 
    #=========================================================================================== 
    def load_parameters(self):
        parameters_file_path = __path__ + '/parameters.json'
        #print parameters_file_path
        with open(parameters_file_path) as json_file:    
            data = json.load(json_file)
        return data

    def call_service(self, service_name):
        print '[%s] service request' % service_name    
        # --- Validate existence for the requested service. --- #
        if service_name in self.services_:
            pass
        else:
            # --- Throw an excetion --- #
            print "Service [%s] does not exist" % service_name

    #===========================================================================================

    def get_platform_services(self):
        return self.services_

    def speech_detection(self, language, audio_source, words, sentences, grammar, fileUri): 

        # -- Craft the data payload for the post request
        payload = {'language':language, 'audio_source':audio_source, \
                'words':json.dumps(words), 'sentences':json.dumps(sentences), \
                'grammar':json.dumps(grammar), 'user': self.user}

        # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['speech_detection']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', \
            'rappdev'))

        resp_msg = json.loads(r.text)
        print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg
    #===========================================================================================


    #===========================================================================================
    def set_denoise_profile(self, noise_audio_fileUri, audio_file_type, user):
        # -- Craft the data payload for the post request
        payload = {'user': user, 'audio_file_type': audio_file_type}

        # -- Files to be added into to poset request
        files = {'noise_audio_fileUri': open(noise_audio_fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['set_denoise_profile']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', \
            'rappdev'))

        resp_msg = json.loads(r.text)
        print  resp_msg
        return resp_msg 
    #===========================================================================================


    #===========================================================================================
    def qr_detection(self, fileUri): 
        # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['qr_detection']
        r = requests.post(url, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        #r = requests.post(url, files=files)

        resp_msg = json.loads(r.text)
        print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg
    #===========================================================================================


    #===========================================================================================
    def face_detection(self, fileUri): 
     # -- Files to be added into to poset request
        files = {'fileUrl': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['face_detection']
        r = requests.post(url, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        #r = requests.post(url, files=files)

        resp_msg = json.loads(r.text)
        #print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg
    #===========================================================================================



    #===========================================================================================
    def ontology_subclassesOf(self, query):
        payload = {'queryStr': query}
        url = self.serviceUrl_['ontology_subclassesOf']
        r = request.post(url, data=payload, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        resp_msg = json.loads(r.text)
        return resp_msg
    #===========================================================================================

    

