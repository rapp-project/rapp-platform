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

# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# contact: klpanagi@gmail.com, etsardou@iti.gr

import os
import rospkg
from os.path import join
import unittest

__path__ = os.path.dirname(os.path.realpath(__file__))

from RappCloud import RappPlatformService
from RappCloud.CloudMsgs import EmailSend
# from RappCloud import RappPlatformAPI


class EmailSendTests(unittest.TestCase):

    def setUp(self):
        self.svc = RappPlatformService()
        self.pkgDir = rospkg.RosPack().get_path('rapp_testing_tools')

    def test_single_attachment(self):
        attach = join(self.pkgDir, 'test_data', 'Lenna.png')

        msg = EmailSend(
            email='rapp.platform@gmail.com',
            password='',
            server='smtp.gmail.com',
            port='587',
            recipients=['rapp.platform@gmail.com'],
            body='Rapp Send Email Test',
            subject='Rapp Send Email Test',
            attach_file=attach)

        resp = self.svc.call(msg)
        self.assertEqual(resp.error, u'')

    def test_no_attachment(self):
        msg = EmailSend(
            email='rapp.platform@gmail.com',
            password='',
            server='smtp.gmail.com',
            port='587',
            recipients=['glagloui@gmail.com'],
            body='Rapp Send Email Test',
            subject='Rapp Send Email Test')
        resp = self.svc.call(msg)
        self.assertEqual(resp.error, u'')

    def test_multiple_files_zip(self):
        attach = join(self.pkgDir, 'test_data', 'zip_files',
                      'image_audio_sample.zip')

        msg = EmailSend(
            email='rapp.platform@gmail.com',
            password='',
            server='smtp.gmail.com',
            port='587',
            recipients=['glagloui@gmail.com'],
            body='Rapp Send Email Test',
            subject='Rapp Send Email Test',
            attach_file=attach)

        resp = self.svc.call(msg)
        self.assertEqual(resp.error, u'')


if __name__ == "__main__":
    unittest.main()
