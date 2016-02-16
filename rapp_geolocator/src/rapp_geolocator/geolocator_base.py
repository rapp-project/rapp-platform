#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Authors: Aris Thallas
# contact: aris.thallas@{iti.gr, gmail.com}


import abc

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError
from rapp_http_request_handler import RappHttpRequestHandler
from rapp_http_json_parser import RappHttpJSONParser


## @class GeolocatorBase
# @brief Base class for geolocators
class GeolocatorBase(object):

    __metaclass__ = abc.ABCMeta

    ## @brief Constructor
    def __init__(self):
        ## The base geolocator url
        self._url = ''

        ## The value of valid response status code
        self._accepted_status = 200

        ## Perform http requests to servers
        self._http_request = RappHttpRequestHandler()

        ## Strips html tags from strings
        self.rapp_http_json_parser = RappHttpJSONParser()

    ## @brief Abstract method to fetch geolocation
    @abc.abstractmethod
    def fetch_geolocation(self, req):
        # This must never be printed
        RappUtilities.rapp_print('Abstract error', 'ERROR')
