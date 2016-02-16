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

import rospy

from geolocator_factory import GeolocatorFactory

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError

from rapp_platform_ros_communications.srv import (
    GeolocatorSrv,
    GeolocatorSrvResponse
    )


## @class GeolocatorNode
# @brief Calculates geolocation via IP
class GeolocatorNode(object):

    ## @brief Constructor
    def __init__(self):
        ## Factory that returns proper geolocator
        self._geolocator_factory = GeolocatorFactory()

        if rospy.has_param('rapp_geolocator_locate_topic'):
            srv_topic = \
                rospy.get_param("rapp_geolocator_locate_topic")
        else:
            srv_topic = ''
            RappUtilities.rapp_print('Geolocator topic not found!', 'ERROR')

        fetch_service = rospy.Service(
            srv_topic, GeolocatorSrv, self.fetch_location_srv_callback
            )

    ## @brief The callback to calculate geolocation
    #
    # @param req
    #   [rapp_platform_ros_communications::Geolocator::GeolocatorSrv]
    #   The service request
    #
    # @return res
    # [rapp_platform_ros_communications::Geolocator::GeolocatorSrvResponse]
    #  The service response
    def fetch_location_srv_callback(self, req):
        response = GeolocatorSrvResponse()

        try:
            locator = \
                self._geolocator_factory.select_geolocator(req.geolocator)
        except RappError as err:
            response.error = str(err)
            return response
        try:
            results = locator.fetch_geolocation(req)
        except RappError as err:
            response.error = str(err)
            return response

        return self._create_service_response(results)

    ## @brief The callback to geolocalize
    #
    # @param results [dict]
    #  The server results containing the location information
    #
    # @return res
    # [rapp_platform_ros_communications::Geolocator::GeolocatorSrvResponse]
    #  The service response
    def _create_service_response(self, result):
        response = GeolocatorSrvResponse()
        response.city = result['city']
        response.country = result['country']
        response.countryCode = result['countryCode']
        response.regionName = result['regionName']
        response.timezone = result['timezone']
        response.zip = result['zip']
        response.longtitude = result['longtitude']
        response.latitude = result['latitude']
        return response


if __name__ == "__main__":
    rospy.init_node('Geolocator')
    geolocator_node = GeolocatorNode()
    RappUtilities.rapp_print("Geolocator node initialized")
    rospy.spin()
