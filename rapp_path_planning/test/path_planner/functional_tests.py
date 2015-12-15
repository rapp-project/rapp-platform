#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest
import rospy
import roslib
import rospkg
import roslaunch
#from geometry_msgs.msg import 
from rapp_platform_ros_communications.srv import (
  PathPlanningRosSrv,
  PathPlanningRosSrvRequest
  )


class PathPlanFunc(unittest.TestCase):

    # def __init__(self, *args, **kwargs):
    #     unittest.TestCase.__init__(self, *args, **kwargs)        
    #     rospy.init_node('rapp_path_planning_test_wrongMap')

    def test_planFound(self):

        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


        req.map_name = "empty"
        req.robot_type = "NAO"
        req.algorithm = "dijkstra"
        #req.start_pose = ()
        #req.goal_pose = ()
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10#rospy.Time.now().secs#.sec = 10#rospy.Time.now().secs
        req.start.header.stamp.nsecs = 10#rospy.Time.now().nsecs
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
        #rospy.init_node('rapp_path_planning_functional_test')
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


        req.map_name = "occupied"
        req.robot_type = "NAO"
        req.algorithm = "dijkstra"
        #req.start_pose = ()
        #req.goal_pose = ()
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10#rospy.Time.now().secs# = 10#rospy.Time.now()
        req.start.header.stamp.nsecs = 10#rospy.Time.now().nsecs
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
        #rospy.init_node('rapp_path_planning_test_getPath')
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


        req.map_name = "empty"
        req.robot_type = "NAO"
        req.algorithm = "dijkstra"
      
        req.start.header.seq = 0
        req.start.header.stamp.nsecs = 10#10#rospy.Time.now().nsecs
        req.start.header.stamp.secs = 0#10#rospy.Time.now().secs# = 10#rospy.Time.now()
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
        #rospy.init_node('rapp_path_planning_test_wrongAlgorithm')
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


        req.map_name = "empty"
        req.robot_type = "NAO"
        req.algorithm = "dikstera" # doeas not exist
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10#rospy.Time.now().secs# = 10#rospy.Time.now()
        req.start.header.stamp.nsecs = 10#rospy.Time.now().nsecs
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
        #rospy.init_node('rapp_path_planning_test_wrongRobot')
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


        req.map_name = "empty"
        req.robot_type = "nao" # doeas not exist
        req.algorithm = "dijkstra"
        #req.start_pose = ()
        #req.goal_pose = ()
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10#rospy.Time.now().secs# = 10#rospy.Time.now()
        req.start.header.stamp.nsecs = 10#rospy.Time.now().nsecs
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
        #rospy.init_node('rapp_path_planning_test_wrongMap')
        rospack = rospkg.RosPack()
        path_service = rospy.get_param("rapp_path_planning_plan_path_topic")
        rospy.wait_for_service(path_service)
        pp_service = rospy.ServiceProxy(path_service, PathPlanningRosSrv)
        req = PathPlanningRosSrvRequest()


        req.map_name = "white_house"# doeas not exist
        req.robot_type = "NAO" 
        req.algorithm = "dijkstra"
        #req.start_pose = ()
        #req.goal_pose = ()
      
        req.start.header.seq = 0
        req.start.header.stamp.secs = 10#rospy.Time.now().secs# = 10#rospy.Time.now()
        req.start.header.stamp.nsecs = 10#rospy.Time.now().nsecs
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














