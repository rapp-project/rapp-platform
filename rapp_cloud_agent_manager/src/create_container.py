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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr
 
import rospy
import os
import sys
from docker import Client
from io import BytesIO
from shutil import copyfile
import ntpath

from cloud_agent_mysql_functions import CloudAgentMysqlFunctions

from rapp_platform_ros_communications.srv import (
  createContainerSrv,
  createContainerSrvResponse  
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )


class CreateContainer:
  
  def createContainer(self,req):
    res = createContainerSrvResponse()
    from docker import Client
    #cli = Client(base_url='tcp://127.0.0.1:2375')
    cli = Client(base_url='unix://var/run/docker.sock')
    
    directory=os.getenv("HOME")+'/rapp_platform_files/tempDockerFileBuilds/'
    if not os.path.exists(directory):
      os.makedirs(directory)
    
    #tmpDirectory=directory=os.getenv("HOME")+'/rapp_platform_files/tempDockerFileBuilds/'+'tmpBuild'
    localFile=ntpath.basename(req.tarUri)
    
    f = open(directory+'/Dockerfile','w')
    f.write('FROM ubuntu\n ADD t.tar.gz /home/\n RUN bash /home/setup.sh\n RUN bash /home/run.sh\n') # python will convert \n to os.linesep
    f.close()
    copyfile(req.tarUri, directory+'/'+localFile)
    
    #dockerfile = '\nFROM ubuntu\n \
    #ADD '+localFile+' /home/\n \
    #RUN ls /home/\n \
    #RUN bash /home/setup.sh\n \
    #RUN bash /home/run.sh'
        
    #print dockerfile
    
    #f = BytesIO(dockerfile.encode('utf-8'))
    response = [line for line in cli.build(path=directory, rm=True, tag='yourname/volume')]
    
    #container = cli.create_container(image='ubuntu', command='/bin/sleep 30')
    print(response)

    print "something" 
    
    return res




