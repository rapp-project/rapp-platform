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
import requests
from rapp_news_explorer.rapp_html_parser import RappHTMLParser
try:
    import simplejson as json
except ImportError:
    import json

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError


## @class NewsEngineBase
# @brief Base class for news engines
class NewsEngineBase(object):

    __metaclass__ = abc.ABCMeta

    ## @brief Constructor
    def __init__(self):
        ## The base news engine url
        self._url = ''

        ## Seconds to wait for any server response
        self._server_timeout = 1.0
        ## The value of valid response status code
        self._accepted_status = 200

        ## Strips html tags from strings
        self._html_parser = RappHTMLParser()

    ## @brief Execute the request and verify response
    #
    # @param header [dict] The request's additional headers dictionary
    #
    # @return response [request module response] The servers response
    #
    # @exceptions RappError Request error
    def perform_request(self, param, header=None):
        if header is None:
            try:
                response = requests.get(
                    self._url, params=param, timeout=self._server_timeout)
            except requests.RequestException as err:
                RappUtilities.rapp_print(err, 'ERROR')
                raise RappError(err)
        else:
            try:
                response = requests.get(
                    self._url, params=param, headers=header,
                    timeout=self._server_timeout)
            except requests.RequestException as err:
                RappUtilities.rapp_print(err, 'ERROR')
                raise RappError(err)

        RappUtilities.rapp_print(response.url, 'DEBUG')

        if (isinstance(self._accepted_status, int) and
                response.status_code == self._accepted_status) or \
            (isinstance(self._accepted_status, list) and
                response.status_code in self._accepted_status):
            return response

        error = 'Request error. Status code: ' + str(response.status_code)
        RappUtilities.rapp_print(error, 'ERROR')
        raise RappError(error)

    ## @brief Abstract method to fetch news from news engine
    @abc.abstractmethod
    def fetch_news(self, req):
        # This must never be printed
        RappUtilities.rapp_print('Abstract error', 'ERROR')
