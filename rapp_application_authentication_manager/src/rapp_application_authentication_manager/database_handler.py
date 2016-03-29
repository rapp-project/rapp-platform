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

    ## Verify that the token exists in store db
    #
    # @param store_token [string] The token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_store_token(self, store_token):
        #  TODO
        pass

    ## Verify that username exists in platform db
    #
    # @param username [string] The username
    #
    # @return status [bool] True if username exists, false otherwise
    def username_exists(self, username):
        #  TODO
        pass

    ## Retrieve user's password from platform db
    #
    # @param username [string] The username
    #
    # @return password [string] Password associated with username
    def get_user_password(self, username):
        #  TODO
        pass

    def verify_active_application_token(self, token):
        #  TODO
        pass

    ## Retrieve username associated with application_token from platform db
    #
    # @param token [string] The user's application token
    #
    # @return username [string] The username
    def get_token_user(self, app_token):
        #  TODO
        pass

    ## Check if there is an active session (token) for the specified user and
    # store_token (device_token)
    #
    # @param username [string] The username
    # @param store_token [string] The device_token associated with the appl_token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_active_robot_session(self, username, store_token):
        #  TODO
        pass

    ## Write new user to the platform db
    #
    # @param username [string] The username
    # @param password [string] The user's password
    # @param store_token [string] The user's store token
    #
    # @return status [bool] True if token exists, false otherwise
    def add_new_user(self, username, password, store_token):
        #  TODO
        pass

    ## Write new token to the platform db
    #
    # @param username [string] The user issuing the token
    # @param store_token [string] The device_token of the application token
    # @param app_token [string] The user's application token
    def write_new_application_token(self, username, store_token, appl_token):
        #  TODO
        pass
