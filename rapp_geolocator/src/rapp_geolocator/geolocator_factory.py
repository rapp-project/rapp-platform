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

from rapp_geolocator.ip_api_locator import IpAPILocator

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError


## @class IPLocatorFactory
# @brief Creates and returns the class of the geolocator requested.
#   Implements a factory pattern.
class GeolocatorFactory(object):

    ## @brief Select proper geolocator according to request
    #
    # @param geolocator [string] The name of the geolocator
    #
    # @return res
    #  [rapp_geolocator.rapp_geolocator.GeolocatorBase]
    #  The geolocator
    #
    # @exceptions RappError Wrong geolocator provided by user
    def select_geolocator(self, geolocator=''):

        # Set google as a default geolocator
        if geolocator == '':
            geolocator = 'ip-api'

        if geolocator == 'ip-api':
            RappUtilities.rapp_print('Creating ip-api locator', 'DEBUG')
            return IpAPILocator()
        else:
            RappUtilities.rapp_print('Wrong ip locator provided', 'ERROR')
            raise RappError('Wrong ip locator provided')
