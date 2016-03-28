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


class DatabaseHandler(object):

    def __init__(self):
        #  TODO: initialize sql wrapper
        pass

    def verify_store_token(self, token):
        #  TODO
        pass

    def username_exists(self, username):
        #  TODO
        pass

    def verify_user_credentials(self, username, password):
        #  TODO
        pass

    def verify_active_application_token(self, token):
        #  TODO

    def verify_active_robot_session(self, username, store_token):
        #  TODO

    def add_new_user(self, username, password):
        #  TODO
       pass
