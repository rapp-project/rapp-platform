#!/usr/bin/env python
# -*- encode: utf-8 -*-

import rospy
from rapp_platform_ros_communications.srv import (
    tutorialExampleServiceSrv,
    tutorialExampleServiceSrvRequest,
    tutorialExampleServiceSrvResponse,
    createOntologyAliasSrv,
    createOntologyAliasSrvRequest,
    createOntologyAliasSrvResponse
)

class AddTwoIntegers:

    def addTwoIntegersFunction(self,req):
        res = tutorialExampleServiceSrvResponse()
        res.additionResult=req.a+req.b
        res.userOntologyAlias=self.getUserOntologyAlias(req.username)
        return res


    def getUserOntologyAlias(self,username):
      serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_ontology_alias')      
      knowrob_service = rospy.ServiceProxy(serv_topic, createOntologyAliasSrv)
      createOntologyAliasReq = createOntologyAliasSrvRequest()
      createOntologyAliasReq.username=username
      createOntologyAliasResponse = knowrob_service(createOntologyAliasReq)
      return createOntologyAliasResponse.ontology_alias
