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

import os
import imaplib
import email

import rospy

from rapp_platform_ros_communications.srv import (
  ReceiveEmailSrv,
  ReceiveEmailSrvResponse
  )


class EmailReceiver(object):

  def __init__(self):
    sendSrvTopic = rospy.get_param("rapp_email_send_topic")
    s = rospy.Service(sendSrvTopic, ReceiveEmailSrv, \
                      self.receiveEmailSrvCallback)


  def receiveEmailSrvCallback(self, req):

    resp = ReceiveEmailSrvResponse()

    userEmail, emailPass = self._getEmailAndPass(req.username)

    try:
      imapConn = self._connectImap(userEmail, emailPass)
    except imaplib.IMAP4.error, err:
      rospy.logerr( str(err) )
      resp.status = -1
      return resp







  ## @brief Fetches user's email and email's password from db
  #
  # @param username [string] The rapp user username
  #
  # @return email    [string] The user's email
  # @return password [string] The user's password
  def _getEmailAndPass(self, username):
    pass

  ## @brief Create an IMAP connection to the server.
  #
  # Initiates an IMAP connection according to the user's email and logins to
  # the server using user's credentials
  #
  # @param email    [string] The user's email
  # @param password [string] The user's password
  #
  # @return imap [imaplib::IMAP4_SSL] The connection
  def _connectImap(self, email, password):

    imapServer = 'imap.' + email.split('@')[2]
    try:
      imap = imaplib.IMAP4_SSL( imapServer )
    except imaplib.IMAP4.error:
      rospy.logerr("Could not establish a connection to the requested IMAP" + \
          ' server: ' + imapServer )
      raise

    try:
      imap = imaplib.IMAP4_SSL( imapServer )
    except imaplib.IMAP4.error:
      rospy.logerr("Could not login to the requested IMAP" + \
          ' server: ' + imapServer )
      raise

    return imap



if __name__ == '__main__':
    rospy.logerr('Implements server for EmailReceiverSrv. Not supposed to' + \
        ' be called directly')
    exit(-1)
