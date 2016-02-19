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

from weather_reporter_factory import WeatherReporterFactory

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError

from rapp_platform_ros_communications.srv import (
    WeatherReporterCurrentSrv,
    WeatherReporterCurrentSrvResponse,
    WeatherReporterForecastSrv,
    WeatherReporterForecastSrvResponse
    )


## @class WeatherReporterNode
# @brief Evaluates weather conditions
class WeatherReporterNode(object):

    ## @brief Constructor
    def __init__(self):
        ## Factory that returns proper weather_reporter
        self._weather_reporter_factory = WeatherReporterFactory()

        if rospy.has_param('rapp_weather_current_topic'):
            srv_topic = \
                rospy.get_param("rapp_weather_current_topic")
        else:
            srv_topic = ''
            RappUtilities.rapp_print('Weather Reporter Current Weather topic not found!', 'ERROR')

        current_weather_service = rospy.Service(
            srv_topic, WeatherReporterCurrentSrv, self.fetch_current_weather_srv_callback
            )

        if rospy.has_param('rapp_weather_forecast_topic'):
            srv_topic = \
                rospy.get_param("rapp_weather_forecast_topic")
        else:
            srv_topic = ''
            RappUtilities.rapp_print('Weather Reporter Forecast topic not found!', 'ERROR')

        forecast_weather_service = rospy.Service(
            srv_topic, WeatherReporterForecastSrv, self.fetch_forecast_srv_callback
            )

    ## @brief The callback to fetch current weather
    #
    # @param req
    #   [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrv]
    #   The service request
    #
    # @return res
    # [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrvResponse]
    #  The service response
    def fetch_current_weather_srv_callback(self, req):
        response = WeatherReporterCurrentSrvResponse()

        try:
            w_reporter = self._weather_reporter_factory.\
                select_weather_reporter(req.weather_reporter)
        except RappError as err:
            response.error = str(err)
            return response
        try:
            results = w_reporter.fetch_current_weather(req)
        except RappError as err:
            response.error = str(err)
            return response

        return self._create_current_service_response(results)

    ## @brief The callback to fetch weather forecast
    #
    # @param req
    #   [rapp_platform_ros_communications::WeatherReporter::WeatherReporterForecastSrv]
    #   The service request
    #
    # @return res
    # [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrvResponse]
    #  The service response
    def fetch_forecast_srv_callback(self, req):
        response = WeatherReporterForecastSrvResponse()

        try:
            w_reporter = self._weather_reporter_factory.\
                select_weather_reporter(req.weather_reporter)
        except RappError as err:
            response.error = str(err)
            return response
        try:
            results = w_reporter.fetch_current_weather(req)
        except RappError as err:
            response.error = str(err)
            return response

        return self._create_current_service_response(results)

    ## @brief Creates proper response
    #
    # @param results [dict]
    #  The server results containing the weather information
    #
    # @return res
    # [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrvResponse]
    #  The service response
    def _create_current_service_response(self, result):
        response = WeatherReporterCurrentSrvResponse()

        response.date = result['date']
        response.temperature = result['temperature']
        response.weather_description = result['weather_description']
        response.humidity = result['humidity']
        response.visibility = result['visibility']
        response.pressure = result['pressure']
        response.wind_speed = result['wind_speed']
        response.wind_temperature = result['wind_temperature']
        response.wind_direction = result['wind_direction']

        return response


if __name__ == "__main__":
    rospy.init_node('WeatherReporter')
    weather_reporter_node = WeatherReporterNode()
    RappUtilities.rapp_print("Weather Reporter node initialized")
    rospy.spin()
