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
    addStoreTokenToDeviceSrv,
    addStoreTokenToDeviceSrvRequest,
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
    checkActiveRobotSessionSrvRequest,
    validateUserRoleSrv,
    validateUserRoleSrvRequest,
    validateExistingPlatformDeviceTokenSrv,
    validateExistingPlatformDeviceTokenSrvRequest
)


## @class DatabaseHandler
# @brief Handles database functions for authentication manager
class DatabaseHandler(object):

    ## @brief ROS Service initializations
    def __init__(self):
        user_exists_topic = rospy.get_param(
            "rapp_mysql_wrapper_check_if_user_exists_service_topic")
        if not user_exists_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_check_if_user_exists_service_topic ' +
                'NOT FOUND', 'ERROR')
        rospy.wait_for_service(user_exists_topic)
        ## Service proxy to query database for user existance
        #
        # (see database_handler.username_exists)
        self._username_exists_proxy = \
            rospy.ServiceProxy(user_exists_topic, checkIfUserExistsSrv,
                               persistent=True)

        get_passwd_topic = rospy.get_param(
            "rapp_mysql_wrapper_get_user_password_service_topic")
        if not get_passwd_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_get_user_password_service_topic ' +
                'NOT FOUND', 'ERROR')
        rospy.wait_for_service(get_passwd_topic)
        ## Service proxy to query database for user password
        #
        # (see database_handler.get_user_password)
        self._get_user_passwd_proxy = \
            rospy.ServiceProxy(get_passwd_topic, getUserPasswordSrv,
                               persistent=True)

        verify_appl_token_topic = rospy.get_param(
            "rapp_mysql_wrapper_check_active_" +
            "application_token_service_topic")
        if not verify_appl_token_topic:
            RappUtilities.rapp_print(
                "rapp_mysql_wrapper_check_active_" +
                "application_token_service_topic NOT FOUND", 'ERROR')
        rospy.wait_for_service(verify_appl_token_topic)
        ## Service proxy to query database for application token validity
        #
        # (see database_handler.verify_active_application_token)
        self._verify_appl_token_proxy = rospy.ServiceProxy(
            verify_appl_token_topic, checkActiveApplicationTokenSrv,
            persistent=True)

        get_token_user_topic = rospy.get_param(
            "rapp_mysql_wrapper_get_username_associated_" +
            "with_application_token_service_topic")
        if not get_token_user_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_get_username_associated_with' +
                '_application_token_service_topic NOT FOUND', 'ERROR')
        rospy.wait_for_service(get_token_user_topic)
        ## Service proxy to query database for the user associated with a token
        #
        # (see database_handler.get_token_user)
        self._get_token_user_proxy = rospy.ServiceProxy(
            get_token_user_topic, getUsernameAssociatedWithApplicationTokenSrv,
            persistent=True)

        verify_robot_session_topic = rospy.get_param(
            "rapp_mysql_wrapper_check_active_" +
            "robot_session_service_topic")
        if not verify_robot_session_topic:
            RappUtilities.rapp_print(
                "rapp_mysql_wrapper_check_active_" +
                "robot_session_service_topic NOT FOUND", 'ERROR')
        rospy.wait_for_service(verify_robot_session_topic)
        ## Service proxy to query database for active user session
        #
        # (see database_handler.verify_active_robot_session)
        self._verify_robot_session_proxy = rospy.ServiceProxy(
            verify_robot_session_topic, checkActiveRobotSessionSrv,
            persistent=True)

        add_new_user_topic = rospy.get_param(
            "rapp_mysql_wrapper_create_new_platform_user_service_topic")
        if not add_new_user_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_create_new_platform_user_service_topic ' +
                'NOT FOUND', 'ERROR')
        rospy.wait_for_service(add_new_user_topic)
        ## Service proxy to add new user to the database
        #
        # (see database_handler.add_new_user)
        self._add_new_user_proxy = \
            rospy.ServiceProxy(add_new_user_topic, createNewPlatformUserSrv,
                               persistent=True)

        create_new_appl_token_topic = rospy.get_param(
            "rapp_mysql_wrapper_create_new_application_token_service_topic")
        if not create_new_appl_token_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_create_new_application_token_' +
                'create_new_appl_token_topic NOT FOUND', 'ERROR')
        rospy.wait_for_service(create_new_appl_token_topic)
        ## Service proxy to add new application token to the database
        #
        # (see database_handler.write_new_application_token)
        self._create_new_app_token_proxy = rospy.ServiceProxy(
            create_new_appl_token_topic, createNewApplicationTokenSrv,
            persistent=True)

        add_store_token_to_device_topic = rospy.get_param(
            "rapp_mysql_wrapper_add_store_token_to_device_topic")
        if not add_store_token_to_device_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_add_store_token_to_device_topic NOT FOUND',
                'ERROR')
        rospy.wait_for_service(add_store_token_to_device_topic)
        ## Service proxy to add new device token from store to the database
        #
        # (see database_handler.add_store_token_to_device)
        self._add_store_token_to_device_proxy = rospy.ServiceProxy(
            add_store_token_to_device_topic, addStoreTokenToDeviceSrv,
            persistent=True)

        validate_user_role_topic = rospy.get_param(
            "rapp_mysql_wrapper_validate_user_role_topic")
        if not validate_user_role_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_validate_user_role_topic NOT FOUND',
                'ERROR')
        rospy.wait_for_service(validate_user_role_topic)
        ## Service proxy to query the database for the user's role
        #
        # (see database_handler.validate_user_role)
        self._validate_user_role_proxy = rospy.ServiceProxy(
            validate_user_role_topic, validateUserRoleSrv,
            persistent=True)

        validate_existing_device_topic = rospy.get_param(
            "rapp_mysql_wrapper_validate_existing_platform_device_token_topic")
        if not validate_existing_device_topic:
            RappUtilities.rapp_print(
                'rapp_mysql_wrapper_validate_' +
                'existing_platform_device_token_topic NOT FOUND',
                'ERROR')
        rospy.wait_for_service(validate_existing_device_topic)
        ## Service proxy to query the database for a device token
        #
        # (see database_handler.verify_platform_device_token)
        self._validate_existing_device_token_proxy = rospy.ServiceProxy(
            validate_existing_device_topic,
            validateExistingPlatformDeviceTokenSrv, persistent=True)

    ## @brief Verify that the device token exists in store database
    #
    # @param device_token [string] The token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_store_device_token(self, device_token):
        #  TODO
        #  pass
        return True

    ## @brief Verify that the device token exists in platform database
    #
    # @param device_token [string] The token
    #
    # @return status [bool] True if token exists, false otherwise
    def verify_platform_device_token(self, device_token):
        req = validateExistingPlatformDeviceTokenSrvRequest()
        req.device_token = device_token

        response = self._validate_existing_device_token_proxy(req)
        return response.success

    ## @brief Verify that username exists in platform database
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

    ## @brief Retrieve user's password from platform database
    #
    # @param username [string] The username
    #
    # @return password [string] Password associated with username
    #
    # @exception RappError Username does not exist
    def get_user_password(self, username):
        req = getUserPasswordSrvRequest()
        req.username = username

        response = self._get_user_passwd_proxy(req)
        if response.success:
            return response.password
        else:
            raise RappError('Wrong credentials')

    ## @brief Check if the application_token is active/valid
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

    ## @brief Retrieve username associated with application_token from platform database
    #
    # @param token [string] The user's application token
    #
    # @return username [string] The username
    def get_token_user(self, app_token):
        req = getUsernameAssociatedWithApplicationTokenSrvRequest()
        req.application_token = app_token

        response = self._get_token_user_proxy(req)
        return response.username

    ## @brief Check if there is an active session (token) for the specified user and
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

    ## @brief Write new user to the platform database
    #
    # @param username [string] The username
    # @param password [string] The user's password
    # @param store_token [string] The user's store token
    # @param language [string] The user's language
    #
    # @return status [bool] True if token exists, false otherwise
    #
    # @exception RappError Failed to create the new user
    def add_new_user(self, username, password, creator, language):
        req = createNewPlatformUserSrvRequest()
        req.username = username
        req.password = password
        req.creator_username = creator
        req.language = language

        response = self._add_new_user_proxy(req)
        if response.success:
            RappUtilities.rapp_print('Succesfully wrote new user', 'DEBUG')
        else:
            RappUtilities.rapp_print(response.error, 'ERROR')
            msg = 'Could not write new user to the database'
            RappUtilities.rapp_print(msg, 'ERROR')
            raise RappError(msg)

    ## @brief Write new token to the platform database
    #
    # @param username [string] The user issuing the token
    # @param store_token [string] The device_token of the application token
    # @param app_token [string] The user's application token
    #
    # @exception RappError Failed to write token
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

    ## @brief Write store token the device table
    #
    # @param store_token [string] The device_token of the application token
    #
    # @exception RappError Failed to write token
    def add_store_token_to_device(self, store_token):
        req = addStoreTokenToDeviceSrvRequest()
        req.store_token = store_token
        res = self._add_store_token_to_device_proxy(req)

        if res.error == '':
            RappUtilities.rapp_print('Succesfully wrote new token', 'DEBUG')
        else:
            RappUtilities.rapp_print(res.error, 'ERROR')
            msg = 'Could not write store token to the database'
            RappUtilities.rapp_print(msg, 'ERROR')
            raise RappError(msg)

    ## @brief Check if user has an admin/root role
    #
    # @param username [string] The user's username
    #
    # @exception RappError Invalid user role
    def validate_user_role(self, username):
        req = validateUserRoleSrvRequest()
        req.username = username
        res = self._validate_user_role_proxy(req)

        if res.error == '':
            RappUtilities.rapp_print('Proper user role', 'DEBUG')
        else:
            #  RappUtilities.rapp_print(res.trace, 'ERROR')
            msg = 'Invalid user role'
            RappUtilities.rapp_print(msg, 'ERROR')
            raise RappError(msg)
