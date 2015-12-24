#! /usr/bin/env python

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
import unittest
import getpass
import random
import string

import roslib
import rospy
import rostest

roslib.load_manifest("rapp_email")

from rapp_platform_ros_communications.srv import (
  SendEmailSrv,
  SendEmailSrvRequest,
  SendEmailSrvResponse
  )

from rapp_platform_ros_communications.msg import (
  MailMsg
  )

class TestEmailSender(unittest.TestCase):
  def setUp(self):
    serviceTopic = rospy.get_param("rapp_email_send_topic")
    rospy.wait_for_service(serviceTopic)

    self._test_service = rospy.ServiceProxy(
        serviceTopic, SendEmailSrv)

    #self._email = raw_input()
    #self._password = getpass.getpass("Enter password: ")
    #self._server = raw_input("Enter server: ")
    #self._port = raw_input("Enter port: ")
    #self._recipient = raw_input("Enter recipient: ")
    #self._attach = raw_input("Enter attachment path: ")

  @unittest.skip('Skipping email tests - Uncomment decorators to test manually')
  def test_checkDefaults(self):

    req = SendEmailSrvRequest()
    req.userEmail = self._email
    req.password = self._password
    req.server = self._server
    req.recipients = []
    req.recipients.append(self._recipient)
    req.body = 'This is my body'
    req.subject = 'This is my subject'
    req.files = []
    req.files.append(self._attach)

    response = self._test_service( req )
    self.assertEquals( response.status, 0 )


  def test_wrongServer(self):

    req = SendEmailSrvRequest()
    req.recipients = []
    req.recipients.append( \
      ''.join( random.choice( string.ascii_lowercase + string.digits ) \
        for _ in range(16)) \
      )
    req.server = \
      ''.join( random.choice( string.ascii_lowercase ) for _ in range(8))

    response = self._test_service( req )
    self.assertEquals( response.status, -1 )

  def test_wrongPort(self):

    req = SendEmailSrvRequest()
    req.recipients = []
    req.recipients.append( \
      ''.join( random.choice( string.ascii_lowercase + string.digits ) \
        for _ in range(16)) \
      )
    req.server = 'smtp.gmail.com'
    req.port = \
      ''.join( random.choice( string.digits ) for _ in range(4))

    response = self._test_service( req )
    self.assertEquals( response.status, -1 )

  def test_wrongUserEmail(self):

    req = SendEmailSrvRequest()
    req.recipients = []
    req.recipients.append( \
      ''.join( random.choice( string.ascii_lowercase + string.digits ) \
        for _ in range(16)) \
      )
    req.server = 'smtp.gmail.com'

    req.userEmail = \
      ''.join( random.choice( string.ascii_lowercase + string.digits ) \
        for _ in range(16))

    response = self._test_service( req )
    self.assertEquals( response.status, -1 )

  def test_wrongPassword(self):

    req = SendEmailSrvRequest()
    req.recipients = []
    req.recipients.append( 'rapp.platform@gmail.com' )
    req.server = 'smtp.gmail.com'

    req.userEmail = 'rapp.platform@gmail.com'
    req.password= \
      ''.join( random.choice( string.ascii_letters + string.digits ) \
        for _ in range(20))

    response = self._test_service( req )
    self.assertEquals( response.status, -1 )

  def test_wrongAttachment(self):

    req = SendEmailSrvRequest()
    req.recipients = []
    req.recipients.append( \
      ''.join( random.choice( string.ascii_lowercase + string.digits ) \
        for _ in range(16)) \
      )
    req.server = 'smtp.gmail.com'

    req.userEmail = 'rapp.platform@gmail.com'
    req.password= \
      ''.join( random.choice( string.ascii_letters + string.digits ) \
        for _ in range(20))

    self.files = []
    req.files.append( \
      ''.join( random.choice( string.ascii_lowercase + string.digits ) \
        for _ in range(16)) \
      )

    response = self._test_service( req )
    self.assertEquals( response.status, -1 )

if __name__ == '__main__':
  rostest.rosrun('rapp_email', 'email_sender_unit_tests', TestEmailSender)
