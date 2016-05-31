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

from eventregistry import *

from rapp_news_explorer.news_engine_base import(
    NewsEngineBase,
    RappUtilities,
    RappError
    )


## @class EventRegistryEngine
# @brief EventRegistry news engine hndler
class EventRegistryEngine(NewsEngineBase):

    ## @brief Constructor
    def __init__(self):
        NewsEngineBase.__init__(self)
        self._event_handler = EventRegistry()

        key_path = os.path.join(os.environ['HOME'],
                                '.config/rapp_platform/api_keys/event_registry')
        if os.path.isfile(key_path):
            RappUtilities.rapp_print('Login', 'WARN')
            with open(key_path) as key_fd:
                self._username = key_fd.readline().strip()
                self._password = key_fd.readline().strip()
                self._event_handler.login(self._username, self._password)
        else:
            RappUtilities.rapp_print('SHIT', 'ERROR')


        # Some default parameter values
        self._max_requests = 20
        self._max_stories = 30

    ## @brief Fetch the news
    #
    # @param req
    #   [rapp_platform_ros_communications::NewsExplorer::NewsExplorerSrv]
    #   The service request
    #
    # @return [list<dict>] The server results containing the stories
    def fetch_news(self, req):
        if req.storyNum < 0:
            error = 'Requested negative number of news stories.'
            RappUtilities.rapp_print(error, 'ERROR')
            raise RappError(error)
        elif req.storyNum == 0:
            warn = 'Requested zero news stories. Providing default number of 5'
            RappUtilities.rapp_print(warn, 'DEBUG')
            req.storyNum = 5

        if req.storyNum < self._max_stories:
            max_stories = req.storyNum
        else:
            warn = 'Too many stories requested. Truncating to: ' + \
                str(self._max_stories)
            RappUtilities.rapp_print(warn, 'DEBUG')
            max_stories = self._max_stories

        max_stories = req.storyNum if req.storyNum < self._max_stories else \
            self._max_stories

        q = QueryArticles()

        for keyword in req.keywords:
            q.addConcept(self._event_handler.getConceptUri(keyword))
        q.addRequestedResult(RequestArticlesInfo(count=max_stories))

        stories = self._event_handler.execQuery(q)

        final_stories = []

        for story in stories['articles']['results']:
            st = {}
            st['title'] = story['title']
            st['content'] = story['body']
            st['publisher'] = story['source']['title']
            st['publishedDate'] = story['date']
            st['url'] = story['url']
            final_stories.append(st)

        return final_stories
