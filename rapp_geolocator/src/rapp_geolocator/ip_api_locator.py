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

from rapp_geolocator.geolocator_base import(
    GeolocatorBase,
    RappUtilities,
    RappError
    )


## @class IpAPILocator
# @brief IP-API locator
class IpAPILocator(GeolocatorBase):

    ## @brief Constructor
    def __init__(self):
        GeolocatorBase.__init__(self)
        self._url = 'http://ip-api.com/json/'

    ## @brief Fetch the location
    #
    # @param req
    #   [rapp_platform_ros_communications::Geolocator::GeolocatorSrv]
    #   The service request
    #
    # @return [dict] The server results
    def fetch_geolocation(self, req):

        try:
            response = self._http_request.perform_request(self._url + req.ip)
        except RappError as err:
            RappUtilities.rapp_print(err, 'ERROR')
            raise RappError(err)

        if response['status'] != 'success':
            err = 'Http request failed: ' + response['message']
            RappUtilities.rapp_print(err, 'ERROR')
            raise RappError(err)

        # Process servers results. Extract titles etc and add to previous
        # results
        try:
            return self._handle_server_response(response)
        except RappError as err:
            RappUtilities.rapp_print(err, 'ERROR')
            raise RappError(err)

    ## @brief Handles the server's response
    #
    # @param response [] The server's response to the request module.
    #
    # @return values [dict] The final values
    def _handle_server_response(self, response):

        # {'server_response_name':'ros_service_name'}
        keys = {'city': 'city',
                'country': 'country',
                'countryCode': 'countryCode',
                'lat': 'latitude',
                'lon': 'longtitude',
                'regionName': 'regionName',
                'timezone': 'timezone',
                'zip': 'zip'}
                #  }

        return self.rapp_http_json_parser.find_values(keys, response)
