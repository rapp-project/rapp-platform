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

roslib.load_manifest("rapp_news_explorer")

from rapp_platform_ros_communications.msg import NewsStoryMsg

from rapp_platform_ros_communications.srv import (
    NewsExplorerSrv,
    NewsExplorerSrvRequest
    )


class TestGoogleNewsExplorer(unittest.TestCase):

    def setUp(self):
        service_topic = rospy.get_param("rapp_news_explorer_fetch_news_topic")

        rospy.wait_for_service(service_topic)

        self._test_service = rospy.ServiceProxy(
            service_topic, NewsExplorerSrv)

    def test_check_default_values(self):
        req = NewsExplorerSrvRequest()
        response = self._test_service(req)
        self.assertEqual(len(response.stories), 5)
        titles = []
        for story in response.stories:
            self.assertNotIn(story.title, titles)
            titles.append(story.title)

    def test_random_keyword(self):
        req = NewsExplorerSrvRequest()
        req.storyNum = 5
        req.keywords.append(
            ''.join(random.choice(string.ascii_letters) for _ in range(10))
            )

        response = self._test_service(req)
        self.assertEqual(len(response.stories), 0)
        self.assertEqual(response.error, "")

    def test_wrong_engine(self):
        req = NewsExplorerSrvRequest()
        req.newsEngine = \
            ''.join(random.choice(string.ascii_letters) for _ in range(10))
        response = self._test_service(req)
        self.assertEqual(len(response.stories), 0)
        self.assertEqual(response.error, "'Wrong news engine provided'")

    def test_too_many_stories(self):
        req = NewsExplorerSrvRequest()
        req.storyNum = 100
        response = self._test_service(req)
        self.assertLess(len(response.stories), req.storyNum)
        self.assertEqual(response.error, '')

    def test_random_region(self):
        req = NewsExplorerSrvRequest()
        req.regionEdition = \
            ''.join(random.choice(string.ascii_letters) for _ in range(10))
        response = self._test_service(req)
        self.assertEqual(len(response.stories), 5)
        self.assertEqual(response.error, '')

    def test_random_topic(self):
        req = NewsExplorerSrvRequest()
        req.topic = \
            ''.join(random.choice(string.ascii_letters) for _ in range(2))
        response = self._test_service(req)
        self.assertNotEqual(response.error, '')

    def test_exclude_files(self):
        req = NewsExplorerSrvRequest()

        # Fetch first batch of news
        response = self._test_service(req)
        self.assertEqual(len(response.stories), 5)
        self.assertEqual(response.error, '')

        # Add first batch of news to the exclude titles
        for story in response.stories:
            req.excludeTitles.append(story.title)

        # Perform second request with exclude titles
        response = self._test_service(req)
        self.assertEqual(len(response.stories), 5)
        self.assertEqual(response.error, '')

        titles = []
        # Verify title uniqueness
        for story in response.stories:
            self.assertNotIn(story.title, titles)
            self.assertNotIn(story.title, req.excludeTitles)
            titles.append(story.title)

if __name__ == '__main__':
    rostest.rosrun(
        'rapp_news_explorer',
        'google_news_explorer_test',
        TestGoogleNewsExplorer)
