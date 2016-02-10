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

from rapp_news_explorer.news_engine_base import(
    NewsEngineBase,
    RappUtilities,
    RappError
    )


## @class GoogleNewsEngine
# @brief Google news engine hndler
class GoogleNewsEngine(NewsEngineBase):

    ## @brief Constructor
    def __init__(self):
        NewsEngineBase.__init__(self)
        self._url = 'https://ajax.googleapis.com/ajax/services/search/news'

        # Some default parameter values
        self._params = {}
        self._params['v'] = '1.0'
        self._params['rsz'] = '8'

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
        if req.storyNum <= 0:
            error = 'Requested negative number of news stories.'
            RappUtilities.rapp_print(error, 'ERROR')
            raise RappError(error)

        max_stories = req.storyNum if req.storyNum < self._max_stories else \
            self._max_stories

        iterations = 0
        final_stories = []

        while iterations < self._max_requests and \
                len(final_stories) < max_stories:

            param_dict = self._handle_params(req, iterations)
            iterations += 1

            # Fetch a number of results from the server
            # (8 per request, due to Google's restrictions)
            try:
                response = self.perform_request(param_dict)
            except RappError as err:
                RappUtilities.rapp_print(err, 'ERROR')
                raise err

            # Process servers results. Extract titles etc and add to previous
            # results
            try:
                self._handle_server_response(
                    response, final_stories, req.excludeTitles)
            except RappError as err:
                RappUtilities.rapp_print(err, 'ERROR')
                raise err

        # Keep the requested number of stories
        final_stories = final_stories[:max_stories]
        return final_stories

    ## @brief Handles the server's response
    #
    # @param response [] The server's response to the request module.
    # @param story_list [string] The current list of stories
    # @param exclude_list [string]
    #   The list of titles to be excluded from the results
    def _handle_server_response(self, response, story_list, exclude_list):
        try:
            response_json = response.json()
        except ValueError as err:
            RappUtilities.rapp_print(err, 'ERROR')
            raise RappError(err)

        for result in response_json['responseData']['results']:
            story = {}

            # Get story title
            self._html_parser.feed(result['titleNoFormatting'])
            story['title'] = self._html_parser.get_data()
            # Skip stories from excluded list
            if story['title'] in exclude_list:
                continue

            # Get story content
            self._html_parser.feed(result['content'])
            story['content'] = self._html_parser.get_data()

            # Get story date
            self._html_parser.feed(result['publishedDate'])
            story['publishedDate'] = \
                self._html_parser.get_data()

            # Get story url
            self._html_parser.feed(result['unescapedUrl'])
            story['url'] = self._html_parser.get_data()

            # Keep unique stories
            story_list.append(story)
            story_list = {v['title']: v for v in story_list}.values()

    ## @brief Create parameter dictionary for request module
    #
    # @param req
    #   [rapp_platform_ros_communications::NewsExplorer::NewsExplorerSrv]
    #   The service request
    #
    # @return params [dict] The parameters
    def _handle_params(self, req, iters):
        params = {}
        query_str = ' '.join(req.keywords)
        if query_str == '':
            if req.topic == '':
                RappUtilities.rapp_print('Wrong query provided.' +
                                         ' Falling back to default topic',
                                         'WARN')
            params['topic'] = 'h'
        else:
            if req.topic != '':
                RappUtilities.rapp_print('Provided both query and topic. ' +
                                         'Ignoring topic',
                                         'WARN')
            params['q'] = query_str

        if req.regionEdition != '':
            params['ned'] = req.regionEdition
        params['start'] = str(iters)

        params.update(self._params)
        return params
