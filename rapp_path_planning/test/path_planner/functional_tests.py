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


class PathPlanFunc(unittest.TestCase):

    def test_planFound(self):

        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


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
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


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
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


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
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


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
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


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
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


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


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'PathPlanFunc', PathPlanFunc)














