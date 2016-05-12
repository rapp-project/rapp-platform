#!/usr/bin/env python

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


PKG='rapp_example_service'

import sys
import unittest
import rospy
import roslib

from rapp_platform_ros_communications.srv import (
    tutorialExampleServiceSrv,
    tutorialExampleServiceSrvRequest,
    tutorialExampleServiceSrvResponse,
  )


class ExampleServiceTests(unittest.TestCase):

    def test_example_service_basic(self):
        ros_service = rospy.get_param(\
                "rapp_example_service_topic")
        rospy.wait_for_service(ros_service)
        
        test_service = rospy.ServiceProxy(\
                ros_service, tutorialExampleServiceSrv)

        req = tutorialExampleServiceSrvRequest()
        req.a=10
        req.b=25       
        req.username="rapp"
        response = test_service(req)
        self.assertEqual(response.userOntologyAlias, "Person_DpphmPqg") 
        self.assertEqual(response.additionResult, 35)        
        
## The main function. Initializes the Cognitive Exercise System functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'ExampleServiceTests', ExampleServiceTests)














