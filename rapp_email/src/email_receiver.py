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
import tempfile
import atexit
import imaplib
import email

import rospy

from rapp_utilities import RappUtilities

from rapp_platform_ros_communications.srv import (
  ReceiveEmailSrv,
  ReceiveEmailSrvResponse
  )


## @class EmailReceiver
# @brief Fetches emails from users email account
class EmailReceiver(object):

  def __init__(self):
    sendSrvTopic = rospy.get_param("rapp_email_send_topic")
    s = rospy.Service(sendSrvTopic, ReceiveEmailSrv, \
                      self.receiveEmailSrvCallback)

  ## The callback to receive specified mails from users email account
  #
  # @rapam req [rapp_platform_ros_communications::Email::ReceiveEmailSrvRequest] The receive email request
  #
  # @rapam res [rapp_platform_ros_communications::Email::ReceiveEmailSrvResponse] The receive email response
  def receiveEmailSrvCallback(self, req):

    resp = ReceiveEmailSrvResponse()
    resp.emails = []

    userEmail, emailPass = self._getEmailAndPass(req.username)

    try:
      imapConn = self._connectImap(userEmail, emailPass)
    except imaplib.IMAP4.error, err:
      resp.status = -1
      return resp

    status, mailNum = imapConn.select()
    if status.lower() != 'OK'.lower():
      RappUtilities.rapp_print("Requested mail folder not found", 'ERROR')
      resp.status = -1
      return resp

    try:
      emailUIDs = self._selectEmails(req, imapConn)
    except imaplib.IMAP4.error, err:
      resp.status = -1
      return resp
    else:
      if len(emailUIDs) == 0:
        RappUtilities.rapp_print('No emails retrieved')
        resp.status = 0
        return resp

    # Fetch the last N requested
    if len(emailUIDs) > req.numberOfEmails:
      emailUIDs = emailUIDs[ len(emailUIDs) - numberOfEmails : ]

    emailPath = self._initializePath(req.username)

    for emailID in emailUIDs:
      self._fetchEmail(imapConn, emailID, emailPath)


  def _fetchEmail(self, imap, emailID, emailPath):
      fetchStatus, data = imap.uid( 'fetch', emailID, '(RFC822)' )
      mail = email.message_from_string( data[0][1] )


  ## Create a temporary path for the user emails
  #
  # The path will be placed in ~/rapp_platform_files/emails/{username}{random_string}
  #
  # @param username [string] The rapp user's username
  def _initializePath(self, username):
    basePath = os.path.join( os.environ['HOME'], 'rapp_platform_files', 'emails' )

    if not os.path.exists( basePath ):
      RappUtilities.rapp_print("Language temporary directory does not exist. " + \
          'Path: ' + basePath)
      os.makedirs( basePath )

    ## The temporary directory containing the configurations
    finalPath = tempfile.mkdtemp( prefix=username, dir = basePath)

    RappUtilities.rapp_print('Email receiver path: ' + finalPath, 'DEBUG')

    # Delete temp file at termination
    atexit.register(shutil.rmtree, finalPath)


  ## @brief Fetch the emails that match the requests criteria
  #
  # @rapam req [rapp_platform_ros_communications::Email::ReceiveEmailSrvRequest] The receive email request
  #
  # @return emails [list<int>] The UIDs of the selected emails
  def _selectEmails(self, req, imap):
    if req.requestedEmailStatus == 'ALL':
      requestedEmailStatus = 'ALL'
    else:
      requestedEmailStatus = 'UNSEEN'
      if req.requestedEmailStatus != 'UNSEEN':
        RappUtilities.rapp_print( \
            'Wrong email status provided. ' + \
            'See EmailReceiveSrv.srv for additional details. ' + \
            'Falling back to default value: "UNSEEN"', 'WARN')

    fromDate = toDate = ''
    if req.fromDate != '':
      fromDate = ' SINCE "' + req.fromDate + '"'
    if req.toDate != '':
      toDate = ' BEFORE "' + req.toDate + '"'

    searchQuery = '(' + requestedEmailStatus + fromDate + toDate + ')'
    RappUtilities.rapp_print(searchQuery)

    try:
      searchStatus, msgIds = imap.uid( 'search', None, searchQuery )
    except imaplib.error, err:
      RappUtilities.rapp_print("Could not perform IMPA search. Query: " + \
          searchQuery, 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise

    msgIds = msgIds[0].split()








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
    except imaplib.IMAP4.error, err:
      RappUtilities.rapp_print( \
          "Could not establish a connection to the requested IMAP server: " + \
          imapServer, 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise

    try:
      imap = imaplib.IMAP4_SSL( imapServer )
    except imaplib.IMAP4.error, err:
      RappUtilities.rapp_print( \
          "Could not login to the requested IMAP server: " + \
          imapServer, 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise

    return imap



if __name__ == '__main__':
    rospy.logerr('Implements server for EmailReceiverSrv. Not supposed to' + \
        ' be called directly')
    exit(-1)
