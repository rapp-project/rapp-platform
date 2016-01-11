#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest
import rospy
import roslib
import rospkg
import roslaunch
from rapp_platform_ros_communications.srv import (
  PathPlanningRosSrv,
  PathPlanningRosSrvRequest
  )
# upload map test
import base64
import shutil
from rapp_platform_ros_communications.srv  import MapServerUploadMapRosSrv, MapServerUploadMapRosSrvRequest
from std_msgs.msg import ByteMultiArray
import os
from os.path import expanduser


class PathPlanFunc(unittest.TestCase):

    def test_planFound(self):

        #rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()

        req.user_name = "functional_test"

        req.map_name = "empty"
        req.robot_type = "NAO"
        req.algorithm = "dijkstra"
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10
        req.start.header.stamp.nsecs = 10
        req.start.header.frame_id = "/map"
        req.goal.header = req.start.header

        req.start.pose.position.x = 1
        req.start.pose.position.y = 1
        req.start.pose.position.z = 0

        req.start.pose.orientation.x = 1
        req.start.pose.orientation.y = 0
        req.start.pose.orientation.z = 0
        req.start.pose.orientation.w = 1

        req.goal.pose.position.x = 5.3
        req.goal.pose.position.y = 4
        req.goal.pose.position.z = 0

        req.goal.pose.orientation = req.start.pose.orientation

        response = pp_service(req)

        plan_found = response.plan_found
        self.assertEqual( plan_found, 1 )

    def test_planNotFound(self):
        #rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()

        req.user_name = "functional_test"

        req.map_name = "occupied"
        req.robot_type = "NAO"
        req.algorithm = "dijkstra"
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10
        req.start.header.stamp.nsecs = 10
        req.start.header.frame_id = "/map"
        req.goal.header = req.start.header

        req.start.pose.position.x = 1
        req.start.pose.position.y = 1
        req.start.pose.position.z = 0

        req.start.pose.orientation.x = 1
        req.start.pose.orientation.y = 0
        req.start.pose.orientation.z = 0
        req.start.pose.orientation.w = 1

        req.goal.pose.position.x = 5.3
        req.goal.pose.position.y = 4
        req.goal.pose.position.z = 0

        req.goal.pose.orientation = req.start.pose.orientation

        response = pp_service(req)

        plan_found = response.plan_found
        self.assertEqual( plan_found, 0 )

    def test_getPath(self):
       # rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()

        req.user_name = "functional_test"

        req.map_name = "empty"
        req.robot_type = "NAO"
        req.algorithm = "dijkstra"
      
        req.start.header.seq = 0
        req.start.header.stamp.nsecs = 10
        req.start.header.stamp.secs = 0
        req.start.header.frame_id = "/map"
        req.goal.header = req.start.header

        req.start.pose.position.x = 1
        req.start.pose.position.y = 1
        req.start.pose.position.z = 0

        req.start.pose.orientation.x = 1
        req.start.pose.orientation.y = 0
        req.start.pose.orientation.z = 0
        req.start.pose.orientation.w = 1

        req.goal.pose.position.x = 5.3
        req.goal.pose.position.y = 4
        req.goal.pose.position.z = 0

        req.goal.pose.orientation = req.start.pose.orientation

        response = pp_service(req)

        self.assertGreater( len(response.path), 1 )

    def test_wrongAlgorithm(self):
        #rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()

        req.user_name = "functional_test"

        req.map_name = "empty"
        req.robot_type = "NAO"
        req.algorithm = "dikstera" # doeas not exist
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10
        req.start.header.stamp.nsecs = 10
        req.start.header.frame_id = "/map"
        req.goal.header = req.start.header

        req.start.pose.position.x = 1
        req.start.pose.position.y = 1
        req.start.pose.position.z = 0

        req.start.pose.orientation.x = 1
        req.start.pose.orientation.y = 0
        req.start.pose.orientation.z = 0
        req.start.pose.orientation.w = 1

        req.goal.pose.position.x = 5.3
        req.goal.pose.position.y = 4
        req.goal.pose.position.z = 0

        req.goal.pose.orientation = req.start.pose.orientation

        response = pp_service(req)

        plan_found = response.plan_found
        print response.error_message

        self.assertEqual( plan_found, 4 )

    def test_wrongRobot(self):
        #rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()

        req.user_name = "functional_test"

        req.map_name = "empty"
        req.robot_type = "nao" # doeas not exist
        req.algorithm = "dijkstra"

        req.start.header.seq = 0
        req.start.header.stamp.secs = 10
        req.start.header.stamp.nsecs = 10
        req.start.header.frame_id = "/map"
        req.goal.header = req.start.header

        req.start.pose.position.x = 1
        req.start.pose.position.y = 1
        req.start.pose.position.z = 0

        req.start.pose.orientation.x = 1
        req.start.pose.orientation.y = 0
        req.start.pose.orientation.z = 0
        req.start.pose.orientation.w = 1

        req.goal.pose.position.x = 5.3
        req.goal.pose.position.y = 4
        req.goal.pose.position.z = 0

        req.goal.pose.orientation = req.start.pose.orientation

        response = pp_service(req)

        plan_found = response.plan_found
        print response.error_message

        self.assertEqual( plan_found, 3 )

    def test_wrongMap(self):
        #rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()

        req.user_name = "functional_test"

        req.map_name = "white_house"# doeas not exist
        req.robot_type = "NAO" 
        req.algorithm = "dijkstra"

        req.start.header.seq = 0
        req.start.header.stamp.secs = 10
        req.start.header.stamp.nsecs = 10
        req.start.header.frame_id = "/map"
        req.goal.header = req.start.header

        req.start.pose.position.x = 1
        req.start.pose.position.y = 1
        req.start.pose.position.z = 0

        req.start.pose.orientation.x = 1
        req.start.pose.orientation.y = 0
        req.start.pose.orientation.z = 0
        req.start.pose.orientation.w = 1

        req.goal.pose.position.x = 5.3
        req.goal.pose.position.y = 4
        req.goal.pose.position.z = 0

        req.goal.pose.orientation = req.start.pose.orientation

        response = pp_service(req)

        plan_found = response.plan_found
        print response.error_message
        self.assertEqual( plan_found, 2 )

    def test_map_upload(self):

        rospack_map = rospkg.RosPack()
        upload_service = rospy.get_param("rapp_path_planning_upload_map_topic")
        rospy.wait_for_service(upload_service)
        um_service = rospy.ServiceProxy(upload_service, MapServerUploadMapRosSrv)
        req = MapServerUploadMapRosSrvRequest()
        map_server_path = rospack_map.get_path("rapp_map_server")

        with open(map_server_path+"/maps/523_m3.png", "rb") as imageFile:
            f = imageFile.read()
            file_bytes = bytes(f)
        up_prox = rospy.ServiceProxy("/rapp/rapp_path_planning/upload_map", MapServerUploadMapRosSrv)
        req = MapServerUploadMapRosSrvRequest()

        req.user_name = "functional_test"
        req.map_name = "523_m_test"
        req.resolution = 0.02
        req.origin = [0.0,0.0,0]
        req.negate = 0
        req.occupied_thresh = 0
        req.free_thresh = 0
        #req.data = b
        bytes_array = ByteMultiArray()
        bytes_array.data = file_bytes
        req.data = bytes_array.data
        req.file_size = len(file_bytes)
        response = up_prox(req)

        status = response.status
        
        self.assertEqual( status, True )

if __name__ == '__main__':
    import rosunit
    if os.path.isdir("/home/rapp/rapp_platform_files/maps/rapp/functional_test"):
        shutil.rmtree("/home/rapp/rapp_platform_files/maps/rapp/functional_test")
    rospack = rospkg.RosPack()
    map_server_path = rospack.get_path("rapp_map_server")
    home = expanduser("~")
    shutil.copytree(map_server_path+"/maps/", home+"/rapp_platform_files/maps/rapp/functional_test/")
    rosunit.unitrun(PKG, 'PathPlanFunc', PathPlanFunc)
    shutil.rmtree("/home/rapp/rapp_platform_files/maps/rapp/functional_test")













