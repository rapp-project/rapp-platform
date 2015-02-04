#!/usr/bin/env python

# MIT License (MIT)

# Copyright (c) <2014> <Rapp Project EU>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import rospy

import smtplib

import email
import email.encoders
import email.mime.text
import email.mime.base

from subprocess import Popen, PIPE

# ROS msgs
from std_msgs.msg import (
    String
)

from rapp_platform_ros_communications.srv import (
    MailSend
)


class Email:

    def __init__(self):
        self.mailSendSrv = rospy.Service("ric/mail/send", MailSend, \
                                         self.mailSendCallback)

    def mailSendCallback(self, req):

        username = req.username.data
        password = req.password.data
        emailMsg = email.MIMEMultipart.MIMEMultipart('alternative')
        emailMsg['Subject'] = req.subject.data
        emailMsg['From'] = req.from_address.data
        emailMsg['To'] = req.to_address.data
        server_name = req.server.data

        if not username:
            username = ""
            password = ""
            emailMsg['From'] = "rapp.platform@gmail.com"
            # Gmail incoming 995, outgoing 465
            server_name = "smtp.gmail.com:587"
    
        # We have an attachment. For MIME bases check here:
        # http://www.freeformatter.com/mime-types-list.html
        if req.attachment_file.data:
            attachmentFile = req.attachment_file.data
            if ".wav" in attachmentFile:
                fileMsg = email.mime.base.MIMEBase('audio', 'x-wav')
                fileMsg.set_payload(file(req.attachment_file.data).read())
                fileMsg.add_header('Content-Disposition', \
                                   'attachment;filename=message.wav')
                email.encoders.encode_base64(fileMsg)

            emailMsg.attach(fileMsg)

        try:
            p = Popen(["/usr/sbin/sendmail", "-t"], stdin=PIPE)
            print emailMsg.as_string()
            p.communicate(emailMsg.as_string())
            
            #rospy.loginfo("Connecting to server...")
            #server = smtplib.SMTP(server_name)
            #rospy.loginfo("Server connected")
            #server.ehlo()
            #server.starttls()
            #rospy.loginfo("Log in with %s", username)
            #server.login(username, password)
            #rospy.loginfo("Log in successful")
            #server.sendmail(emailMsg['From'], emailMsg['To'], \
                            #emailMsg.as_string())
            #rospy.loginfo("Mail sent")
        except smtplib.socket.gaierror:
            return String("Couldn't contact the host")
        except smtplib.SMTPAuthenticationError:
            return String("Login failed")
        #finally:
            #if server:
                #server.quit()

        return String("Success")

if __name__ == '__main__':
    rospy.init_node('mail_ros_node')
    mail_node = Email()
    rospy.spin()
