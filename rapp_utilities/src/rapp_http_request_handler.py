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


import requests

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError


## @class RappHttpRequestHandler
# @brief Performs http requests
class RappHttpRequestHandler(object):

    def __init__(self):
        ## The server url
        self._url = ''
        ## The request parameters
        self._params = {}
        ## The request additional headers
        self._headers = {}

        ## The required response format
        self._response_format = 'json'

        ## Seconds to wait for any server response
        self._server_timeout = 1.0
        ## The value of valid response status code
        self._accepted_status = 200

    ## @brief Execute the request and verify response
    #
    # @param url_add [string] The server url
    # @param params_dict [dict] The request's parameters
    # @param headers_dict [dict] The request's additional headers dictionary
    #
    # @return response [request module response] The servers response
    #
    # @exceptions RappError Request error
    def perform_request(self,
                        url_add=None,
                        params_dict=None,
                        headers_dict=None):

        #  Handle arguments
        url = self._url
        if url_add is not None:
            url = url_add
        param = self._params.copy()
        if params_dict is not None:
            param.update(params_dict)
        head = self._headers.copy()
        if headers_dict is not None:
            head.update(headers_dict)

        try:
            response = requests.get(url, params=param,
                                    headers=head, timeout=self._server_timeout)
        except requests.RequestException as err:
            RappUtilities.rapp_print(err, 'ERROR')
            raise RappError(err)

        if (isinstance(self._accepted_status, int) and
                response.status_code == self._accepted_status) or \
            (isinstance(self._accepted_status, list) and
                response.status_code in self._accepted_status):
            return self._modify_response(response)

        error = 'Request error. Status code: ' + str(response.status_code)
        RappUtilities.rapp_print(error, 'ERROR')
        raise RappError(error)

    ## @brief Return the proper response format
    #
    # @param response [request module response] The servers response
    # @param form [string] The required response format
    #
    # @exceptions RappError Format transformation error
    def _modify_response(self, response, form=None):
        if form is None and self._response_format == 'json' or form == 'json':
            try:
                return response.json()
            except ValueError as err:
                RappUtilities.rapp_print(err, 'ERROR')
                raise RappError(err)
        else:
            return response.text()
