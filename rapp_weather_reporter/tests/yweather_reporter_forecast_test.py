#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import unittest
import random
import string

import roslib
import rospy
import rostest

roslib.load_manifest("rapp_weather_reporter")

from rapp_platform_ros_communications.srv import (
    WeatherReporterForecastSrv,
    WeatherReporterForecastSrvRequest
    )


class TestForecastYWeatherReporter(unittest.TestCase):

    def setUp(self):
        self._cities = ['Thessaloniki', 'Oslo', 'Madrid', 'New York']

        service_topic = rospy.get_param("rapp_weather_forecast_topic")

        rospy.wait_for_service(service_topic)

        self._test_service = rospy.ServiceProxy(
            service_topic, WeatherReporterForecastSrv)

    #  @unittest.expectedFailure
    @unittest.skip('Skipping tests - Uncomment decorators to test manually')
    def test_check_default_values(self):
        req = WeatherReporterForecastSrvRequest()
        response = self._test_service(req)
        self.assertEqual(response.error,
            "'City provided is wrong or not supported'")

    #  @unittest.expectedFailure
    @unittest.skip('Skipping tests - Uncomment decorators to test manually')
    def test_actual_city(self):
        req = WeatherReporterForecastSrvRequest()
        for city in self._cities:
            req.city = city
            response = self._test_service(req)
            self.assertEqual(response.error, "")
            self.assertEqual(len(response.forecast), 5)

    #  @unittest.expectedFailure
    @unittest.skip('Skipping tests - Uncomment decorators to test manually')
    def test_random_city(self):
        req = WeatherReporterForecastSrvRequest()
        req.city = \
            ''.join(random.choice(string.ascii_letters) for _ in range(20))
        response = self._test_service(req)
        self.assertEqual(response.error,
            "'City provided is wrong or not supported'")

if __name__ == '__main__':
    rostest.rosrun(
        'rapp_weather_reporter',
        'yweather_reporter_forecast_test',
        TestForecastYWeatherReporter)
