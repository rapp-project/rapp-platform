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

roslib.load_manifest("rapp_geolocator")

from rapp_platform_ros_communications.srv import (
    GeolocatorSrv,
    GeolocatorSrvRequest
    )


class TestIpApiGeolocator(unittest.TestCase):

    def setUp(self):
        service_topic = rospy.get_param("rapp_geolocator_locate_topic")

        rospy.wait_for_service(service_topic)

        self._test_service = rospy.ServiceProxy(
            service_topic, GeolocatorSrv)

    def test_check_default_values(self):
        req = GeolocatorSrvRequest()
        response = self._test_service(req)
        self.assertEqual(response.error, "'No IP provided'")

    def test_wrong_geolocator(self):
        req = GeolocatorSrvRequest()
        req.geolocator = \
            ''.join(random.choice(string.ascii_letters) for _ in range(10))
        response = self._test_service(req)
        self.assertEqual(response.error, "'Wrong ip locator provided'")

    def test_right_ip(self):
        req = GeolocatorSrvRequest()
        req.ip = '155.209.15.20'
        response = self._test_service(req)
        self.assertEqual(response.error, '')

    def test_bad_ip(self):
        req = GeolocatorSrvRequest()
        req.ip = '955.209.15.20'
        response = self._test_service(req)
        self.assertIn("'Http request failed: invalid query'",
            response.error)

if __name__ == '__main__':
    rostest.rosrun(
        'rapp_geolocator',
        'ip_api_geolocator_test',
        TestIpApiGeolocator)
