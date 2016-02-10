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

import rospy

from rapp_news_explorer.engine_factory import EngineFactory

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError

from rapp_platform_ros_communications.srv import (
    NewsExplorerSrv,
    NewsExplorerSrvResponse
    )


## @class NewsExplorerNode
# @brief Fetches news from various News sites' APIs
class NewsExplorerNode(object):

    ## @brief Constructor
    def __init__(self):
        ## Factory that returns proper news engine
        self._engine_factory = EngineFactory()

        if rospy.has_param('rapp_news_explorer_fetch_news_topic'):
            srv_topic = \
                rospy.get_param("rapp_news_explorer_fetch_news_topic")
        else:
            srv_topic = ''
            RappUtilities.rapp_print('Fetch News topic not found!', 'ERROR')

        fetch_service = rospy.Service(
            srv_topic, NewsExplorerSrv, self.fetch_news_srv_callback
            )

    ## @brief The callback to fetch news
    #
    # @param req
    #   [rapp_platform_ros_communications::NewsExplorer::NewsExplorerSrv]
    #   The service request
    #
    # @return res
    # [rapp_platform_ros_communications::NewsExplorer::NewsExplorerSrvResponse]
    #  The service response
    def fetch_news_srv_callback(self, req):
        response = NewsExplorerSrvResponse()
        try:
            engine = self._engine_factory.select_news_engine(req.newsEngine)
        except RappError as err:
            response.error = err
            return response


if __name__ == "__main__":
    rospy.init_node('NewsExplorer')
    news_explorer_node = NewsExplorerNode()
    RappUtilities.rapp_print("News Explorer node initialized")
    rospy.spin()
