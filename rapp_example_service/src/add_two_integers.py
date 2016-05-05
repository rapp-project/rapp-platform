#!/usr/bin/env python
# -*- encode: utf-8 -*-

import rospy
from rapp_platform_ros_communications.srv import (
    tutorialExampleServiceSrv,
    tutorialExampleServiceSrvRequest,
    tutorialExampleServiceSrvResponse
)

class AddTwoIntegers:

    def addTwoIntegersFunction(self,req):
        res = tutorialExampleServiceSrvResponse()
        res.additionResult=req.a+req.b
        return res

