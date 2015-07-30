#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
from requests.auth import HTTPBasicAuth
import json
import os

#  Set and hold RappCloud directory path
__path__ = os.path.dirname(__file__)


class RappCloud:

    ##
    #   @brief default constructor
    ##
    def __init__(self):
        # --- Load Rapp Platform parameters --- #
        params = self.load_parameters()
        self.platform_params_ = params['platform']
        # ------------------------------------- #

        self.platformIP_ = self.platform_params_['host_ip']
        self.services_ = []
        self.serviceUrl_ = {}
        self.auth_ = {}
        self.servicePort_ = self.platform_params_['services']['port']
        self.auth_['user'] = self.platform_params_['auth']['username']
        self.auth_['passwd'] = self.platform_params_['auth']['password']

        for service in self.platform_params_['services']['name']:
            self.services_.append(service)
            self.serviceUrl_[service] = 'http://' + self.platformIP_ + ':' + str(self.servicePort_) + '/hop/' + \
                service
    #===========================================================================================


    ##
    #   @brief load server parameters from parameters.json file
    ##
    def load_parameters(self):
        parameters_file_path = __path__ + '/../parameters.json'
        #print parameters_file_path
        with open(parameters_file_path) as json_file:
            data = json.load(json_file)
        return data


    ##
    #   @brief Call different services throught a single method
    #   @TODO Implement!!!
    ##
    def call_service(self, service_name, args):
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

    def speech_detection_sphinx4(self, language, audio_source, words, sentences, grammar, fileUri, user):

        # -- Craft the data payload for the post request
        payload = {'language':language, 'audio_source':audio_source, \
                'words':json.dumps(words), 'sentences':json.dumps(sentences), \
                'grammar':json.dumps(grammar), 'user': user}

        # -- Files to be added into to poset request
        files = {'file_uri': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['speech_detection_sphinx4']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', \
            'rappdev'))
        resp_msg = json.loads(r.text)
        return resp_msg
    #===========================================================================================


    #===========================================================================================
    def set_denoise_profile(self, noise_audio_fileUri, audio_file_type, user):
        # -- Craft the data payload for the post request
        payload = {'user': user, 'audio_source': audio_file_type}

        # -- Files to be added into to poset request
        files = {'file_uri': open(noise_audio_fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['set_denoise_profile']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', \
            'rappdev'))

        resp_msg = json.loads(r.text)
        #print  resp_msg
        return resp_msg
    #===========================================================================================


    #===========================================================================================
    def qr_detection(self, fileUri):
        # -- Files to be added into to poset request
        files = {'file_uri': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['qr_detection']
        r = requests.post(url, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        #r = requests.post(url, files=files)

        resp_msg = json.loads(r.text)
        #print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg
    #===========================================================================================


    #===========================================================================================
    def face_detection(self, fileUri):
     # -- Files to be added into to poset request
        files = {'file_uri': open(fileUri, 'rb')}

        # -- Post-Request!
        url = self.serviceUrl_['face_detection']
        r = requests.post(url, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        #r = requests.post(url, files=files)

        resp_msg = json.loads(r.text)
        #print '\033[1;33m' + str(resp_msg) + '\033[0m'
        return resp_msg
    #===========================================================================================



    #===========================================================================================
    def ontology_subclasses_of(self, query):
        payload = {'query': query}
        url = self.serviceUrl_['ontology_subclasses_of']
        r = requests.post(url, data=payload, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        resp_msg = json.loads(r.text)
        return resp_msg
    #===========================================================================================


    def ontology_superclasses_of(self, query):
        payload = {'query': query}
        url = self.serviceUrl_['ontology_superclasses_of']
        r = requests.post(url, data=payload, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        resp_msg = json.loads(r.text)
        return resp_msg
    #===========================================================================================

    def ontology_is_subsuperclass_of(self, parent_class, child_class, recursive):
        rec = False
        if recursive != None:
          rec = recursive

        payload = {'parent_class': parent_class, 'child_class': child_class, \
          'recursive': rec}
        url = self.serviceUrl_['ontology_is_subsuperclass_of']
        r = requests.post(url, data=payload, auth=HTTPBasicAuth('rappdev', 'rappdev'))
        resp_msg = json.loads(r.text)
        return resp_msg
    #===========================================================================================

    def detect_objects(self, fileUri, limit):
        # -- Files to be added into to poset request
        files = {'file_uri': open(fileUri, 'rb')}
        payload = {'limit': int(limit)}

        # -- Post-Request!
        url = self.serviceUrl_['image_recognition']
        r = requests.post(url, data=payload, files=files, auth=HTTPBasicAuth('rappdev', 'rappdev'))

        resp_msg = json.loads(r.text)
        return resp_msg
    #===========================================================================================
