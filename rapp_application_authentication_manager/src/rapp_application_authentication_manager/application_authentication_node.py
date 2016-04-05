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

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError

from rapp_platform_ros_communications.srv import (
    AddNewUserFromPlatformSrv,
    AddNewUserFromPlatformSrvResponse,
    AddNewUserFromStoreSrv,
    AddNewUserFromStoreSrvResponse,
    UserTokenAuthenticationSrv,
    UserTokenAuthenticationSrvResponse,
    UserLoginSrv,
    UserLoginSrvResponse
    )


class ApplicationAuthenticationManager:

    def __init__(self):

        self._db_handler = DatabaseHandler()

        # Create new user using platfrom credentials
        rapp_add_new_user_from_platform_topic = \
            rospy.get_param("rapp_add_new_user_from_platform_topic")
        if not rapp_add_new_user_from_platform_topic:
            msg = "Add new user from platform topic does not exist"
            RappUtilities.rapp_print(msg, 'ERROR')
        add_new_user_from_platform_srv = \
            rospy.Service(rapp_add_new_user_from_platform_topic,
                          AddNewUserFromPlatformSrv,
                          self.add_new_user_from_platform_callback)

        # Create new user using store credentials
        rapp_add_new_user_from_store_topic = \
            rospy.get_param("rapp_add_new_user_from_store_topic")
        if not rapp_add_new_user_from_store_topic:
            msg = "Add new user from store topic does not exist"
            RappUtilities.rapp_print(msg, 'ERROR')
        add_new_user_srv = rospy.Service(rapp_add_new_user_from_store_topic,
                                         AddNewUserFromStoreSrv,
                                         self.add_new_user_from_store_callback)

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

        # Login using sore credentials
        login_from_store_topic = \
            rospy.get_param("rapp_login_user_from_store")
        if not login_from_store_topic:
            msg = "Login user from store topic does not exist"
            RappUtilities.rapp_print(msg, 'ERROR')
        login_user_from_store_srv = \
            rospy.Service(login_from_store_topic,
                          UserLoginSrv,
                          self.login_from_store_callback)

        # Login
        login_topic = \
            rospy.get_param("rapp_login_user")
        if not login_topic:
            msg = "Login user topic does not exist"
            RappUtilities.rapp_print(msg, 'ERROR')

        login_user_srv = \
            rospy.Service(login_topic,
                          UserLoginSrv,
                          self.login_callback)

    # TODO: Desc
    def add_new_user_from_platform_callback(self, req):
        pass

    # The token generation service callback
    def add_new_user_from_store_callback(self, req):
        res = AddNewUserSrvResponse()

        # Verify that username -> alphanumeric + dash + underscore
        if not re.match("^[\w\d_-]*$", req.username) :
            res.error = 'Invalid username characters'
            return res

        if not self._db_handler.verify_store_token(req.device_token):
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

        # TODO: Add password restrictions (length etc.)
        password_hash = bcrypt.hashpw(req.password, bcrypt.gensalt())

        try:
            self._db_handler.add_new_user(
                req.username, password_hash, req.device_token, req.language)
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
        res.username = ''
        if self._db_handler.verify_active_application_token(req.token):
            res.username = self._db_handler.get_token_user(req.token)
        else:
            res.error = 'Invalid token'
        return res

    def login_callback(self, req):
        pass

    # User login service callback
    def login_from_store_callback(self, req):
        res = UserLoginSrvResponse()

        try:
            password = self._db_handler.get_user_password(req.username)
        except RappError as e:
            res.error = 'Wrong credentials'
            return res
        if bcrypt.hashpw(req.password, password) != password:
            res.error = 'Wrong credentials'
            return res

        # TODO: verify that device_token is actual and active store token
        if not self._db_handler.verify_store_token(req.device_token):
            res.error = 'Invalid user'
            return res

        if self._db_handler.verify_active_robot_session(req.username, req.device_token):
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
                req.username, req.device_token, res.token)
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
