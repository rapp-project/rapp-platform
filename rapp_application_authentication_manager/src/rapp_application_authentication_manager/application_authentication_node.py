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

import re
import random
import string
import bcrypt
import base64
from passlib.hash import sha256_crypt

import rospy

from database_handler import DatabaseHandler
from rapp_exceptions import RappError

from rapp_platform_ros_communications.srv import (
    AddNewUserSrv,
    AddNewUserSrvResponse,
    UserTokenAuthenticationSrv,
    UserTokenAuthenticationSrvResponse,
    UserLoginSrv,
    UserLoginSrvResponse
    )


class ApplicationAuthenticationManager:

    def __init__(self):

        self._db_handler = DatabaseHandler()

        # Token generation service
        rapp_add_new_user_topic = rospy.get_param("rapp_add_new_user_topic")
        if not rapp_add_new_user_topic:
            rospy.logerr("Application authentication: " +
                         "Token generation topic does not exist")
        add_new_user_srv = rospy.Service(rapp_add_new_user_topic,
                                         AddNewUserSrv,
                                         self.add_new_user_callback)

        # Token authentication service
        authenticate_token_topic = \
            rospy.get_param("rapp_authenticate_token_topic")
        if not authenticate_token_topic:
            rospy.logerr("Application authentication: " +
                         "Token authentication topic does not exist")

        authenticate_token_service = \
            rospy.Service(authenticate_token_topic,
                          UserTokenAuthenticationSrv,
                          self.authenticate_token_callback)

        # Login
        login_topic = \
            rospy.get_param("rapp_login_user")
        if not login_topic:
            rospy.logerr("Application authentication: " +
                         "Login topic does not exist")

        login_user_srv = rospy.Service(
            login_topic, UserLoginSrv, self.login_callback)

    # The token generation service callback
    def add_new_user_callback(self, req):
        res = AddNewUserSrvResponse()

        # Verify that username -> alphanumeric + dash + underscore
        if not re.match("^[\w\d_-]*$", req.username) :
            res.error = 'Invalid username characters'
            return res

        if not self._db_handler.verify_store_token(req.user_token):
            res.error = 'Invalid user'
            return res

        if self._db_handler.username_exists(req.username):
            res.error = 'Username exists'
            counter = 0
            while True and counter <= 10:
                counter += 1
                suggestion = '_' + \
                    ''.join(random.SystemRandom().choice(string.digits)
                            for _ in range(5))
                if not self._db_handler.username_exists(
                        req.username + suggestion):
                    res.suggested_username = req.username + suggestion
                    return res
            return res

        password_hash = bcrypt.hashpw(req.password, bcrypt.gensalt())
        #  RappUtilities.rapp_print(password_hash, 'WARN')

        try:
            self._db_handler.add_new_user(
                req.username, password_hash, req.user_token, req.language)
        except RappError as e:
            res.error = 'Error'
        else:
            res.error = ''
        finally:
            return res

    # The token authentication service callback
    def authenticate_token_callback(self, req):

        res = UserTokenAuthenticationSrvResponse()
        res.error = ''
        res.authenticated = ''
        if self._db_handler.verify_active_application_token(req.token):
            res.username = self._db_handler.get_token_user(req.token)
        else:
            res.error = 'Invalid token'
        return res

    # User login service callback
    def login_callback(self, req):
        res = UserLoginSrvResponse()

        try:
            password = self._db_handler.get_user_password(req.username)
        except RappError as e:
            res.error = 'Wrong credentials'
            return res
        if bcrypt.hashpw(req.password, password) != password:
            res.error = 'Wrong credentials'
            return res


        if self._db_handler.verify_active_robot_session(req.username, req.user_token):
            # TODO: verify that user_token is actual and active store token
            res.error = 'Session already active'
            return res

        #  Generate token
        rand_str = \
            ''.join(random.SystemRandom().choice(string.ascii_letters +
                    string.digits + string.punctuation) for _ in range(64))
        hash_str = sha256_crypt.encrypt(rand_str)
        index = hash_str.find('$', 3)
        hash_str = hash_str[index+1:]
        res.token = base64.b64encode(hash_str)

        try:
            self._db_handler.write_new_application_token(
                req.username, req.user_token, res.token)
        except RappError as e:
            res.error = 'Wrong credentials'
        else:
            res.error = ''
        finally:
            return res

if __name__ == "__main__":
    rospy.init_node('application_authentication_node')
    application_authentication_ros_node = ApplicationAuthenticationManager()
    rospy.spin()
