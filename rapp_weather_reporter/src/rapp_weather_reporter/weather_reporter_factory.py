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

from rapp_weather_reporter.yweather_reporter import YWeatherReporter

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError


## @class WeatherReporterFactory
# @brief Creates and returns the class of the weather reporter requested.
#   Implements a factory pattern.
class WeatherReporterFactory(object):

    ## @brief Select proper weather reporter according to request
    #
    # @param weather_reporter [string] The name of the weather reporter 
    #
    # @return res
    #  [rapp_weather_reporter.rapp_weather_reporter.WeatherReporterBase]
    #  The weather reporter 
    #
    # @exceptions RappError Wrong weather reporter provided by user
    def select_weather_reporter(self, weather_reporter=''):

        # Set yweather(Yahoo) as a default weather reporter 
        if weather_reporter == '':
            weather_reporter = 'yweather'

        if weather_reporter == 'yweather':
            RappUtilities.rapp_print('Creating yweather weather reporter', 'DEBUG')
            return YWeatherReporter()
        else:
            RappUtilities.rapp_print('Wrong weather reporter provided', 'ERROR')
            raise RappError('Wrong weather reporter provided')
