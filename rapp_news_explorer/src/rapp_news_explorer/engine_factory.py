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

from rapp_news_explorer.google_news_engine import GoogleNewsEngine

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError


## @class EngineFactory
# @brief Creates and returns the class of the news engine requested.
#   Implements a factory pattern.
class EngineFactory(object):

    ## @brief Select proper news engine according to request
    #
    # @param engine [string] The name of the news engine
    #
    # @return res
    #  [rapp_news_explorer.rapp_news_explorer.NewsEngine]
    #  The news engine
    #
    # @exceptions RappError Wrong news engine provided by user
    def select_news_engine(self, engine):

        # Set google as a default news engine
        if engine == '':
            engine = 'google'

        if engine == 'google':
            RappUtilities.rapp_print('Creating Google News engine', 'DEBUG')
            return GoogleNewsEngine()
        else:
            RappUtilities.rapp_print('Wrong news engine provided', 'ERROR')
            raise RappError('Wrong news engine provided')
