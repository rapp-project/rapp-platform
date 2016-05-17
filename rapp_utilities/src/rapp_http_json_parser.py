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

from rapp_html_parser import RappHTMLParser


## @class RappHttpJSONParser
# @brief Retrieves key:value pairs from JSON responses
class RappHttpJSONParser(object):

    ## @brief Constructor
    def __init__(self):
        self._html_parser = RappHTMLParser()

    ## @brief Creates key:value pairs from JSON responses
    #
    # Accepts a json_key:final_key dictionary and returns a dictionary with
    # values final_key[i]:value[i] where json_key[i]:final_key[i] and
    # json_key[i]:value[i]
    #
    # @param key_dict [dictionary] The keys dictionary
    # @param json_data [dictionary] The JSON data dictionary
    #
    # @return result [dictionary] The final dictionary
    def find_values(self, key_dict, json_data):
        result = {}

        for key in key_dict:
            if isinstance(json_data[key], str):
                self._html_parser.feed(json_data[key])
                result[key_dict[key]] = self._html_parser.get_data()
            else:
                result[key_dict[key]] = json_data[key]

        return result
