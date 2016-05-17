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
import datetime
import tempfile
import shutil
import atexit
import imaplib
import socket
import email

import rospy

from rapp_utilities import RappUtilities

from rapp_platform_ros_communications.srv import (
  ReceiveEmailSrv,
  ReceiveEmailSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  MailMsg
  )


## @class EmailReceiver
# @brief Fetches emails from user's email account
class EmailReceiver(object):

  ## Constructor
  def __init__(self):
    receiveSrvTopic = rospy.get_param("rapp_email_receive_topic")
    receiveSrv = rospy.Service(receiveSrvTopic, ReceiveEmailSrv, \
                      self.receiveEmailSrvCallback)

  ## The callback to receive specified mails from users email account
  #
  # @rapam req [rapp_platform_ros_communications::Email::ReceiveEmailSrvRequest] The receive email request
  #
  # @rapam res [rapp_platform_ros_communications::Email::ReceiveEmailSrvResponse] The receive email response
  def receiveEmailSrvCallback(self, req):

    resp = ReceiveEmailSrvResponse()
    resp.emails = []

    RappUtilities.rapp_print('Request:\n' + str(req), 'DEBUG')
    try:
      imapConn = self._connectImap(req.email, req.password, req.server, req.port)
    except (imaplib.IMAP4.error, socket.error), err:
      resp.status = -1
      return resp

    # Select inbox
    status, mailNum = imapConn.select()
    if status.lower() != 'OK'.lower():
      RappUtilities.rapp_print("Requested mail folder not found", 'ERROR')
      resp.status = -1
      return resp

    try:
      emailUIDs = self._selectEmails(req, imapConn)
      RappUtilities.rapp_print('# of email selected to fetch: ' + \
          str(len(emailUIDs)), 'DEBUG')
    except imaplib.IMAP4.error, err:
      resp.status = -1
      return resp
    else:
      if len(emailUIDs) == 0:
        RappUtilities.rapp_print('No emails retrieved')
        resp.status = 0
        return resp

    # Fetch the last N requested
    if req.numberOfEmails != 0 and len(emailUIDs) > req.numberOfEmails:
      emailUIDs = emailUIDs[ len(emailUIDs) - req.numberOfEmails : ]

    emailPath = self._initializePath(req.email)

    emailCounter = 1
    for emailID in emailUIDs:
      emailData = self._fetchEmail(imapConn, emailID, emailPath, emailCounter)
      resp.emails.append(emailData)
      emailCounter += 1

    return resp

  ## Fetch specified email's data
  #
  # @param imap [imaplib::IMAP4_SSL] The connection
  # @param emailID [int] The emails UID
  # @param receivePath [string] The absolute path where the contents of the email should be written.
  # @param emailCounter [int] The number of emails processed
  #
  # return mailMsg [rapp_platform_ros_communications::MailMsg] The mail msg to be appended in the service response
  def _fetchEmail(self, imap, emailID, receivePath, emailCounter):
    mailMsg = MailMsg()

    fetchStatus, data = imap.uid( 'fetch', emailID, '(RFC822)' )
    mail = email.message_from_string( data[0][1] )

    # Create temporary directory for the email
    emailPath = tempfile.mkdtemp( prefix = str(emailCounter) + '_', \
        dir = receivePath)

    # Create temporary file for the email's body
    bodyFD, bodyFilePath = tempfile.mkstemp( prefix='body_', \
        dir = emailPath )
    # Compose email's header
    os.write( bodyFD, \
      'From: ' + str(mail['From']) + '\n' + \
      'To: ' + str(mail['To']) + '\n' + \
      'CC: ' + str(mail['cc']) + '\n' + \
      'Subject: ' + str(mail['Subject']) + '\n' \
      'Date: ' + mail['Date'] + '\n\n' \
    )

    # Create response's email element
    mailMsg.bodyPath = bodyFilePath
    mailMsg.subject = str(mail['Subject'])
    mailMsg.dateTime = str(mail['Date'])
    mailMsg.sender = str(mail['From'])
    mailMsg.receivers = []
    for receiver in str(mail['To']).split(','):
      mailMsg.receivers.append(receiver.strip())

    if mail['cc'] is not None:
      for receiver in str(mail['cc']).split(','):
        mailMsg.receivers.append(receiver.strip())
    mailMsg.attachmentPaths = []

    attachmentCounter = 1
    #Iterate emails parts to get attachments
    for part in mail.walk():
      if part.get_content_maintype() == 'multipart':
        continue
      if part.get_content_maintype() == 'text':
        # Get body text and discard text attachments
        if part.get('Content-Disposition') is None and \
            part.get_content_type() != 'text/html':
          os.write(bodyFD, part.get_payload( decode = True ) )
          os.write(bodyFD, '\n' )
          continue

      if part.get('Content-Disposition') is None:
          continue

      # This is an attachment
      filename = part.get_filename()
      if not filename:
        filename = 'attachment-%03d' % attachmentCounter
        attachmentCounter += 1

      # Create temporary file for the email's attachment
      attachmentFD, attachmentPath = tempfile.mkstemp( prefix=filename + "_", \
          dir = emailPath )

      os.write(attachmentFD, part.get_payload( decode = True ) )
      os.close(attachmentFD)
      mailMsg.attachmentPaths.append( attachmentPath )

    os.close(bodyFD)
    return mailMsg


  ## Create a temporary path for the user emails
  #
  # The path will be placed in ~/rapp_platform_files/emails/{username}{random_string}
  #
  # @param email [string] The rapp user's email
  def _initializePath(self, email):
    basePath = os.path.join( os.environ['HOME'], 'rapp_platform_files', 'emails' )

    if not os.path.exists( basePath ):
      RappUtilities.rapp_print("Language temporary directory does not exist. " + \
          'Path: ' + basePath)
      os.makedirs( basePath )

    username = email.split('@')[0]
    ## The temporary directory containing the configurations
    finalPath = tempfile.mkdtemp( prefix=username + '_', dir = basePath)

    RappUtilities.rapp_print('Email receiver path: ' + finalPath, 'DEBUG')

    # Delete temp file at termination
    # TODO: Check whether files are preserved long enough for the agent to collect
    atexit.register(shutil.rmtree, finalPath)
    return finalPath


  ## @brief Fetch the emails that match the requests criteria
  #
  # @rapam req [rapp_platform_ros_communications::Email::ReceiveEmailSrvRequest] The receive email request
  #
  # @return emails [list<int>] The UIDs of the selected emails
  #
  # @except imaplib.IMAP4.error General imaplib error
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
    if req.fromDate != 0:
      dateString = datetime.datetime.fromtimestamp(req.fromDate).strftime( \
          '%d-%b-%Y' )
      fromDate = ' SINCE "' + dateString + '"'
    if req.toDate != 0:
      dateString = datetime.datetime.fromtimestamp(req.toDate).strftime( \
          '%d-%b-%Y' )
      toDate = ' BEFORE "' + dateString + '"'

    searchQuery = '(' + requestedEmailStatus + fromDate + toDate + ')'
    RappUtilities.rapp_print(searchQuery, 'DEBUG')

    try:
      searchStatus, msgIds = imap.uid( 'search', None, searchQuery )
    except imaplib.IMAP4.error, err:
      RappUtilities.rapp_print("Could not perform IMPA search. Query: " + \
          searchQuery, 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise

    return msgIds[0].split()


  ## @brief Create an IMAP connection to the server.
  #
  # Initiates an IMAP connection according to the user's email and logins to
  # the server using user's credentials
  #
  # @param email    [string] The user's email
  # @param password [string] The user's password
  # @param server   [string] The email server
  # @param port     [string] The email server's port
  #
  # @return imap [imaplib::IMAP4_SSL] The connection

  # @except imaplib.IMAP4.error General imaplib error
  # @exception socket.error Socket class exception (connection problem)
  def _connectImap(self, email, password, server, port):

    try:
      socket.setdefaulttimeout(5)
      if port is not None and port != '':
        imap = imaplib.IMAP4_SSL( server, port )
      else:
        imap = imaplib.IMAP4_SSL( server )
    except (imaplib.IMAP4.error, socket.error), err:
      RappUtilities.rapp_print( \
          "Could not establish a connection to the requested IMAP server: " + \
          server + ' port: ' + port, 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise err

    try:
      imap.login( email, password )
    except imaplib.IMAP4.error, err:
      RappUtilities.rapp_print( \
          "Could not login to the requested IMAP server: " + \
          server, 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise err

    return imap


if __name__ == '__main__':
  print('Implements server for EmailReceiverSrv. Not supposed to' + \
      ' be called directly')
  exit(-1)
