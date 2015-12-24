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
import smtplib
import socket
import mimetypes
from email import encoders
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.audio import MIMEAudio
from email.mime.image import MIMEImage
from email.mime.base import MIMEBase

import rospy

from rapp_utilities import RappUtilities

from rapp_platform_ros_communications.srv import (
  SendEmailSrv,
  SendEmailSrvResponse
  )

class EmailSender(object):

  def __init__(self):
    sendSrvTopic = rospy.get_param("rapp_email_send_topic")
    sendSrv = rospy.Service(sendSrvTopic, SendEmailSrv, \
                      self.sendEmailSrvCallback)

  ## The callback to send specified mails from users email account
  #
  # @rapam req [rapp_platform_ros_communications::Email::SendEmailSrvRequest] The send email request
  #
  # @rapam res [rapp_platform_ros_communications::Email::SendEmailSrvResponse] The send email response
  def sendEmailSrvCallback(self, req):

    resp = SendEmailSrvResponse()

    RappUtilities.rapp_print(req, 'DEBUG')

    if len(req.recipients) == 0:
      RappUtilities.rapp_print("Must provide at least one recipient", 'ERROR')
      resp.status = -1
      return resp

    msg = self._createEmailBody( \
        req.recipients, req.userEmail, req.body, req.subject )

    for attachment in req.files:
      try:
        attach = self._handleAttachments( attachment )
      except EnvironmentError, err:
        RappUtilities.rapp_print("Failed to handle attachment: " + attachment, \
            'ERROR')
        RappUtilities.rapp_print(err, 'ERROR')
        resp.status = -1
        return resp
      else:
        msg.attach( attach )

    try:
      self._connectAndSend( req.userEmail, req.password, req.recipients, \
          req.server, req.port, msg )
    except (socket.error, smtplib.SMTPException):
      resp.status = -1
      return resp

    resp.status = 0
    return resp


  def _connectAndSend(self, userEmail, userPassword, recipients, server, port, msg):
    RappUtilities.rapp_print( "Connecting to the requested SMTP server: " + \
        server + ' port: ' + port)
    try:
      socket.setdefaulttimeout(5)
      if port is not None and port != '':
        smtpServer = smtplib.SMTP(server, port)
      else:
        smtpServer = smtplib.SMTP(server)
    except socket.error, err:
      RappUtilities.rapp_print( \
          "Could not establish a connection to the requested SMTP server: " + \
          server + ' port: ' + port, 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise err

    try:
      smtpServer.ehlo()
      smtpServer.starttls()
      smtpServer.ehlo()
      smtpServer.login( userEmail, userPassword )
      smtpServer.sendmail( userEmail, recipients, msg.as_string() )
    except smtplib.SMTPException, err:
      RappUtilities.rapp_print( "SMTP failed!", 'ERROR')
      RappUtilities.rapp_print( err, 'ERROR')
      raise err
    finally:
      smtpServer.quit()

  def _createEmailBody( self, recipients, userEmail, body, subject ):

    if subject == '' or subject is None:
      RappUtilities.rapp_print('No email subject provided')
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = userEmail
    msg['To'] = ', '.join( recipients )
    if body == '' or body is None:
      RappUtilities.rapp_print('No email body provided')
    msg.attach( MIMEText(body) )
    return msg

  def _handleAttachments( self, filename ):
    #TODO: sanitize paths

    if not os.path.isfile( filename ):
      raise IOError('Filename does not exist! ' + filename)

    fp = open( filename , 'rb')
    filetype, encoding = mimetypes.guess_type( fp.name )
    if filetype is None or encoding is not None:
      filetype = 'application/octet-stream'
    maintype, subtype = filetype.split('/', 1)

    RappUtilities.rapp_print('Attachment : ' + filename )
    RappUtilities.rapp_print('Attachment maintype: ' + maintype + ' subtype: ' +\
        subtype)
    if maintype == 'text':
      attachment = MIMEText( fp.read(), _subtype=subtype )
    elif maintype == 'image':
      attachment = MIMEImage( fp.read(), _subtype=subtype )
    elif maintype == 'audio':
      attachment = MIMEAudio( fp.read(), _subtype=subtype )
    else:
      attachment = MIMEBase( maintype, subtype )
      attachment.set_payload( fp.read() )
      encoders.encode_base64( attachment )

    fp.close()

    attachment.add_header( 'Content-Disposition',\
        'attachment', filename = os.path.basename(filename) )
    return attachment

if __name__ == '__main__':
  rospy.logerr('Implements server for EmailSendSrv. Not supposed to' + \
      ' be called directly')
  exit(-1)
