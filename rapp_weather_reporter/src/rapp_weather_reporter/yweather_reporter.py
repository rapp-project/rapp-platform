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

import yweather
import urllib2
import xml

from rapp_weather_reporter.weather_reporter_base import(
    WeatherReporterBase,
    RappUtilities,
    RappError
    )


## @class YWeatherReporter
# @brief Yweather weather reporter
class YWeatherReporter(WeatherReporterBase):

    ## @brief Constructor
    def __init__(self):
        WeatherReporterBase.__init__(self)
        self._client = yweather.Client()


    def _fetch_city_id(self, city_str):
        counter = 4
        while counter != 0:
            try:
                city_id = self._client.fetch_woeid(city_str)
            except urllib2.URLError as err:
                RappUtilities.rapp_print(err, 'ERROR')
                counter -= 1
            except ( xml.etree.ElementTree.ParseError) as err:
                RappUtilities.rapp_print(err, 'ERROR')
                raise RappError(err)
            else:
                if city_id is None:
                    counter -= 1
                else:
                    return city_id

        if city_id is None:
            err = 'City provided is wrong or not supported'
        else:
            err = 'Yweather server error'
        RappUtilities.rapp_print(err, 'ERROR')
        raise RappError(err)


    def _fetch_yweather_report(self, city_id, param_metric='True'):
        try:
            return self._client.fetch_weather(city_id, metric=param_metric)
        except (urllib2.URLError, xml.etree.ElementTree.ParseError) as err:
            RappUtilities.rapp_print(err, 'ERROR')
            raise RappError(err)

    ## @brief Fetch the current weather
    #
    # @param req
    #   [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrv]
    #   The service request
    #
    # @return [dict] The server results
    def fetch_current_weather(self, req):

        city_id = self._fetch_city_id(req.city)
        weather_xml = self._fetch_yweather_report(city_id, req.metric)
        return self._handle_current_weather_report(weather_xml)

    ## @brief Fetch the current weather
    #
    # @param req
    #   [rapp_platform_ros_communications::WeatherReporter::WeatherReporterCurrentSrv]
    #   The service request
    #
    # @return [dict] The server results
    def fetch_weather_forecast(self, req):
        city_id = self._fetch_city_id(req.city)
        weather_xml = self._fetch_yweather_report(city_id, req.metric)
        return self._handle_weather_forecast_report(weather_xml)

    ## @brief Handles the server's response
    #
    # @param response [] The server's response to the request module.
    #
    # @return values [dict] The final values
    def _handle_current_weather_report(self, report):
        response = {}

        response['date'] = report['lastBuildDate']
        response['temperature'] = report['condition']['temp']
        response['weather_description'] = report['condition']['text']
        response['humidity'] = report['atmosphere']['humidity']
        response['visibility'] = report['atmosphere']['visibility']
        response['pressure'] = report['atmosphere']['pressure']
        response['wind_speed'] = report['wind']['speed']
        response['wind_temperature'] = report['wind']['chill']
        response['wind_direction'] = report['wind']['compass']

        return response

    def _handle_weather_forecast_report(self, report):
        response = []
        for forecast in report['forecast']:
            fore = {}
            fore['high_temperature'] = forecast['high']
            fore['low_temperature'] = forecast['low']
            fore['description'] = forecast['text']
            fore['date'] = forecast['date']
            response.append(fore)
        return response
