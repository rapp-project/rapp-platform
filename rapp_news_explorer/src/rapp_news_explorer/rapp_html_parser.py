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
# Source: http://stackoverflow.com/a/925630

from HTMLParser import HTMLParser


## @class RappHTMLParser
# @brief Strips html from imput string
class RappHTMLParser(HTMLParser):

    ## @brief Constructor
    def __init__(self):
        HTMLParser.__init__(self)
        self.reset()
        self.fed = []

    ## @brief Handles the provided string
    #
    # @param input_str [string] The string containing html tags
    def handle_data(self, input_str):
        self.fed.append(input_str)

    ## @brief Returns the processed string
    #
    # @return final_str [string] The string without the html tags
    def get_data(self):
        final_str = ''.join(self.fed)
        self.fed = []
        return final_str
