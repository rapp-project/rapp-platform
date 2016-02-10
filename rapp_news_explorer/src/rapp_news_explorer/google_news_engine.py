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

from rapp_news_explorer.news_engine_base import NewsEngineBase


## @class GoogleNewsEngine
# @brief Google news engine hndler
class GoogleNewsEngine(NewsEngineBase):

    def __init__(self):
        NewsEngineBase.__init__(self)
        self._url = 'https://ajax.googleapis.com/ajax/services/search/news'

        # Some default parameter values
        self._params = {}
        self._params['v'] = '1.0'

    def fetch_news(self, req):
        self._handle_params(req)

    def _handle_params(self, req):
        params = {}
        if len(req.keywords) == 0:
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
            params['q'] = ' '.join(req.keywords)

        if req.regionEdition != '':
            params['ned'] = req.regionEdition

        params.update(self._params)
        return params
