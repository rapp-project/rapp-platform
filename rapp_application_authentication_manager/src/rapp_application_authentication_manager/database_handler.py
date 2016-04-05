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

# Authors: Aris Thallas, Athanassios Kintsakis
# contact: aris.thallas@{iti.gr, gmail.com}

import rospy

from rapp_utilities import RappUtilities
from rapp_exceptions import RappError

from rapp_platform_ros_communications.srv import (
    getUserOntologyAliasSrv,
    getUserOntologyAliasSrvRequest,
    registerUserOntologyAliasSrv,
    registerUserOntologyAliasSrvRequest,
    checkIfUserExistsSrv,
    checkIfUserExistsSrvRequest,
    getUserLanguageSrv,
    getUserLanguageSrvRequest,
    getUserPasswordSrv,
    getUserPasswordSrvRequest,
    getUsernameAssociatedWithApplicationTokenSrv,
    getUsernameAssociatedWithApplicationTokenSrvRequest,
    createNewPlatformUserSrv,
    createNewPlatformUserSrvRequest,
    createNewApplicationTokenSrv,
    createNewApplicationTokenSrvRequest,
    checkActiveApplicationTokenSrv,
    checkActiveApplicationTokenSrvRequest,
    checkActiveRobotSessionSrv,
    checkActiveRobotSessionSrvRequest
)


class DatabaseHandler(object):

    def __init__(self):
        #  TODO: initialize sql wrapper
        user_exists_topic = rospy.get_param(
            "rapp_mysql_wrapper_check_if_user_exists_service_topic")
        if not user_exists_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_check_if_user_exists_service_topic ' +
                'NOT FOUND', 'ERROR')
        rospy.wait_for_service(user_exists_topic)
        self._username_exists_proxy = \
            rospy.ServiceProxy(user_exists_topic, checkIfUserExistsSrv)

        get_passwd_topic = rospy.get_param(
            "rapp_mysql_wrapper_get_user_password_service_topic")
        if not get_passwd_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_get_user_password_service_topic ' +
                'NOT FOUND', 'ERROR')
        rospy.wait_for_service(get_passwd_topic)
        self._get_user_passwd_proxy = \
            rospy.ServiceProxy(get_passwd_topic, getUserPasswordSrv)

        verify_appl_token_topic = rospy.get_param(
            "rapp_mysql_wrapper_check_active_" +
            "application_token_service_topic")
        if not verify_appl_token_topic:
            RappUtilities.rapp_print(
                "rapp_mysql_wrapper_check_active_" +
                "application_token_service_topic NOT FOUND", 'ERROR')
        rospy.wait_for_service(verify_appl_token_topic)
        self._verify_appl_token_proxy = rospy.ServiceProxy(
            verify_appl_token_topic, checkActiveApplicationTokenSrv)

        get_token_user_topic = rospy.get_param(
            "rapp_mysql_wrapper_get_username_associated_" +
            "with_application_token_service_topic")
        if not get_token_user_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_get_username_associated_with' +
                '_application_token_service_topic NOT FOUND', 'ERROR')
        rospy.wait_for_service(get_token_user_topic)
        self._get_token_user_proxy = rospy.ServiceProxy(
            get_token_user_topic, getUsernameAssociatedWithApplicationTokenSrv)

        verify_robot_session_topic = rospy.get_param(
            "rapp_mysql_wrapper_check_active_" +
            "robot_session_service_topic")
        if not verify_robot_session_topic:
            RappUtilities.rapp_print(
                "rapp_mysql_wrapper_check_active_" +
                "robot_session_service_topic NOT FOUND", 'ERROR')
        rospy.wait_for_service(verify_robot_session_topic)
        self._verify_robot_session_proxy = rospy.ServiceProxy(
            verify_robot_session_topic, checkActiveRobotSessionSrv)

        add_new_user_topic = rospy.get_param(
            "rapp_mysql_wrapper_create_new_platform_user_service_topic")
        if not add_new_user_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_create_new_platform_user_service_topic ' +
                'NOT FOUND', 'ERROR')
        rospy.wait_for_service(add_new_user_topic)

        self._add_new_user_proxy = \
            rospy.ServiceProxy(add_new_user_topic, createNewPlatformUserSrv)

        create_new_apll_token_topic = rospy.get_param(
            "rapp_mysql_wrapper_create_new_application_token_service_topic")
        if not create_new_apll_token_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_create_new_application_token_' +
                'create_new_apll_token_topic NOT FOUND', 'ERROR')
        rospy.wait_for_service(create_new_apll_token_topic)

        self._create_new_app_token_proxy = rospy.ServiceProxy(
            create_new_apll_token_topic, createNewApplicationTokenSrv)

    ## Verify that the token exists in store db
    #
    # @param store_token [string] The token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_store_token(self, store_token):
        #  TODO
        #  pass
        return True

    ## Verify that username exists in platform db
    #
    # @param username [string] The username
    #
    # @return status [bool] True if username exists, false otherwise
    def username_exists(self, username):
        req = checkIfUserExistsSrvRequest()
        req.username = username

        response = self._username_exists_proxy(req)
        if response.success:
            return response.user_exists
        else:
            return False

    ## Retrieve user's password from platform db
    #
    # @param username [string] The username
    #
    # @return password [string] Password associated with username
    def get_user_password(self, username):
        req = getUserPasswordSrvRequest()
        req.username = username

        response = self._get_user_passwd_proxy(req)
        if response.success:
            return response.password
        else:
            raise RappError('Password fetch failed')

    ## Check if the application_token is active/valid
    #
    # @param token [string] The user's application token
    #
    # @return status [bool] True if it is valid, flase otherwise
    def verify_active_application_token(self, application_token):

        req = checkActiveApplicationTokenSrvRequest()
        req.application_token = application_token

        response = self._verify_appl_token_proxy(req)
        if response.success:
            return response.application_token_exists
        else:
            return False

    ## Retrieve username associated with application_token from platform db
    #
    # @param token [string] The user's application token
    #
    # @return username [string] The username
    def get_token_user(self, app_token):
        req = getUsernameAssociatedWithApplicationTokenSrvRequest()
        req.application_token = app_token

        response = self._get_token_user_proxy(req)
        return response.username

    ## Check if there is an active session (token) for the specified user and
    # store_token (device_token)
    #
    # @param username [string] The username
    # @param store_token [string] The device_token associated with the appl_token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_active_robot_session(self, username, store_token):
        req = checkActiveRobotSessionSrvRequest()
        req.username = username
        req.device_token = store_token

        response = self._verify_robot_session_proxy(req)
        if response.success:
            return response.application_token_exists
        else:
            return False

    ## Write new user to the platform db
    #
    # @param username [string] The username
    # @param password [string] The user's password
    # @param store_token [string] The user's store token
    # @param language [string] The user's language
    #
    # @return status [bool] True if token exists, false otherwise
    def add_new_user(self, username, password, store_token, language):
        req = createNewPlatformUserSrvRequest()
        req.username = username
        req.password = password
        req.store_token = store_token
        req.language = language

        response = self._add_new_user_proxy(req)
        if response.success:
            RappUtilities.rapp_print('Succesfully wrote new user', 'DEBUG')
        else:
            msg = 'Could not write new user to the database'
            RappUtilities.rapp_print(msg, 'ERROR')
            raise RappError(msg)

    ## Write new token to the platform db
    #
    # @param username [string] The user issuing the token
    # @param store_token [string] The device_token of the application token
    # @param app_token [string] The user's application token
    def write_new_application_token(self, username, store_token, appl_token):
        req = createNewApplicationTokenSrvRequest()
        req.username = username
        req.store_token = store_token
        req.application_token = appl_token

        response = self._create_new_app_token_proxy(req)
        if response.success:
            RappUtilities.rapp_print('Succesfully wrote new token', 'DEBUG')
        else:
            msg = 'Could not write new application token to the database'
            RappUtilities.rapp_print(msg, 'ERROR')
            raise RappError(msg)
