#!/usr/bin/env python
# -*- encode: utf-8 -*-

import rospy
from add_two_integers import AddTwoIntegers
from rapp_platform_ros_communications.srv import (
    tutorialExampleServiceSrv,
    tutorialExampleServiceSrvRequest,
    tutorialExampleServiceSrvResponse
)

class ExampleService:

    def __init__(self): 
        self.serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')
        if(not self.serv_topic):
          rospy.logerror("rapp_knowrob_wrapper_create_ontology_alias param not found")
        rospy.wait_for_service(self.serv_topic)
      
        self.serv_topic = rospy.get_param("rapp_example_service_topic")
        if(not self.serv_topic):
          rospy.logerror("rapp_example_service_topic")
        self.serv=rospy.Service(self.serv_topic, tutorialExampleServiceSrv, self.tutorialExampleServiceDataHandler)

    def tutorialExampleServiceDataHandler(self,req):
        res = tutorialExampleServiceSrvResponse()
        it = AddTwoIntegers()
        res=it.addTwoIntegersFunction(req)
        return res   

