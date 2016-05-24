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

import os
from forecastiopy import *
import geocoder

from rapp_weather_reporter.weather_reporter_base import(
    WeatherReporterBase,
    RappUtilities,
    RappError
    )


## @class ForecastIOReporter
# @brief ForecastIO weather reporter
class ForecastIOReporter(WeatherReporterBase):

    ## @brief Constructor
    def __init__(self):
        WeatherReporterBase.__init__(self)
        key_path = os.path.join(os.environ['HOME'],
                                '.config/rapp_platform/api_keys/forecast.io')
        with open(key_path) as key_fd:
            self._api_key = key_fd.readline().strip()

    ## @brief Perfom request for weather data
    #
    # @param city [string] The city name
    #
    # @return fio [object] The weather report object
    def _get_fio_object(self, city):
        geocode = geocoder.google(city)

        fio = ForecastIO.ForecastIO(self._api_key,
                                    latitude=geocode.latlng[0],
                                    longitude=geocode.latlng[1])
        return fio

    ## @brief Fetch the current weather
    #
    # @param req
    #   [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrv]
    #   The service request
    #
    # @return [dict] The server results
    def fetch_current_weather(self, req):
        fio = self._get_fio_object(req.city)
        if fio.has_currently() is True:
            currently = FIOCurrently.FIOCurrently(fio)
            return self._handle_current_weather_report(currently)
        else:
            raise RappError('Could not fetch current weather')

    ## @brief Fetch the current weather
    #
    # @param req
    #   [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrv]
    #   The service request
    #
    # @return [dict] The server results
    def fetch_weather_forecast(self, req):
        fio = self._get_fio_object(req.city)
        if fio.has_daily() is True:
            daily = FIODaily.FIODaily(fio)
            return self._handle_weather_forecast_report(daily)
        else:
            raise RappError('Could not fetch weather forecast')

    ## @brief Handles the server's response
    #
    # @param response [] The server's response to the request module.
    #
    # @return values [dict] The final values
    def _handle_current_weather_report(self, report):
        response = {}

        response['date'] = str(report.time)
        response['temperature'] = str(report.temperature)
        response['weather_description'] = str(report.summary)
        response['humidity'] = str(report.humidity)
        response['visibility'] = str(report.visibility)
        response['pressure'] = str(report.pressure)
        response['wind_speed'] = str(report.windSpeed)
        response['wind_temperature'] = ''
        response['wind_direction'] = ''

        return response

    def _handle_weather_forecast_report(self, report):
        response = []
        for day in xrange(0, report.days()):
            fore = {}
            fore['high_temperature'] = \
                str(report.get_day(day)['temperatureMax'])
            fore['low_temperature'] = \
                str(report.get_day(day)['temperatureMin'])
            fore['description'] = str(report.get_day(day)['summary'])
            fore['date'] = str(report.get_day(day)['time'])
            response.append(fore)
        return response
