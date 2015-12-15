#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
__author__ = "Wojciech Dudek"


import rospy
import sys
import signal
import roslaunch
import yaml
from nav_fn.srv import MakePlan
from rapp_platform_ros_communications.srv import PathPlanningRosSrv, PathPlanningRosSrvResponse

class ToDictionaty:
    def __init__(self, **entries): 
        self.__dict__.update(entries)

class ServiceHandling():
	def __init__(self):
		rospy.init_node('rapp_path_planning_service_handler')
		self.initializeService()
		
	def initializeService(self):
		rospy.Service('/rapp_path_planning_node/plann_path', PathPlanningRosSrv , self.service_handler)

	def service_handler(self,req):
		path_planner = PathPlanning()
		algorithm_config = path_planner.setPlanningAlgorithm(req.algorithm)
		robot_type_config = path_planner.setCostmapConfig(req.robot_type)
		path_planner.initializeGlobalPlanner(req.map_path, robot_type_config, algorithm_config)
		path = path_planner.plannPath(req.start, req.finish)
		path_planner.killNodes()
		return PathPlanningRosSrvResponse(path)

class PathPlanning():
	def __init__(self):
		rospy.init_node('rapp_path_planning_node', anonymous=True)
		self.node_name = rospy.get_name()
		# remove '/' sign at the beginning of node_name string
		self.node_name = name[1:]

	def publishCostmapParams(ParamObj):
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/global_frame', ParamObj.global_frame)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/robot_base_frame'ParamObj.robot_base_frame)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/update_frequency'ParamObj.update_frequency)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/publish_frequency'ParamObj.publish_frequency)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/plugins',ParamObj.plugins)
		#set if you want the voxel map published
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/publish_voxel_map',ParamObj.publish_voxel_map)
		#set to true if you want to initialize the costmap from a static map
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/static_map',ParamObj.static_map)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/map_topic','map_server'+'/'+str(self.node_name)+'/map')
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/unknown_cost_value',ParamObj.unknown_cost_value)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/lethal_cost_threshold',ParamObj.lethal_cost_threshold)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/footprint',ParamObj.footprint)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/footprint_padding',ParamObj.footprint_padding)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/costmap/cost_scaling_factor',ParamObj.cost_scaling_factor)

	def publishPlannerParams(ParamObj):
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/planner/allow_unknown', ParamObj.allow_unknown)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/planner/default_tolerance'ParamObj.default_tolerance)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/planner/visualize_potential'ParamObj.visualize_potential)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/planner/use_dijkstra'ParamObj.use_dijkstra)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/planner/use_quadratic',ParamObj.use_quadratic)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/planner/use_grid_path',ParamObj.use_grid_path)
		rospy.set_param('global_planner'+'/'+str(self.node_name)+'/planner/old_navfn_behavior',ParamObj.old_navfn_behavior)

	def parseYAMLtoObject(YAML_path):
		import yaml
		f = open(YAML_path)
		dataMap = yaml.safe_load(f)
		f.close()
		Yaml_data_object = ToDictionaty(**dataMap)
		return Yaml_data_object

	def initializeGlobalPlanner(self,map_path,cost_map_YAML_path,planner_YAML_path):
		
		package_MS = 'map_server'
		executable_MS = 'map_server'
		node_name_MS = 'map_server'+'/'+str(self.node_name)
		node_MS = roslaunch.core.Node(package_MS, executable_MS)
		node_MS.name = node_name_MS
		node_MS.args = map_path
		launch_MS = roslaunch.scriptapi.ROSLaunch()
		launch_MS.start()
		self.process_MS = launch_MS.launch(node_MS)

		package_tf = 'tf'
		executable_tf = 'static_transform_publisher'
		node_name_tf = 'base_link_broadcaser'
		node_tf = roslaunch.core.Node(package_tf, executable_tf)
		node_tf.name = node_name_tf
		node_tf.args = "0.066 1.7 0.54 0.49999984  0.49960184  0.49999984  0.50039816 map base_link 100"
		launch_tf = roslaunch.scriptapi.ROSLaunch()
		launch_tf.start()
		self.process_tf = launch_tf.launch(node_tf)

		package_GP = 'global_planner'
		executable_GP = 'planner'
		node_name_GP = 'global_planner'+'/'+str(self.node_name)
		node_GP = roslaunch.core.Node(package_GP, executable_GP)
		node_GP.name = node_name_GP
		launch_GP = roslaunch.scriptapi.ROSLaunch()
		launch_GP.start()
		self.process_GP = launch_GP.launch(node_GP)

		rospy.set_param(node_name_GP+'/use_sim_time','false')

		cost_map_YAML_obj = self.parseYAMLtoObject(cost_map_YAML_path)
		planner_YAML_obj = self.parseYAMLtoObject(planner_YAML_path)

		self.publishCostmapParams(cost_map_YAML_obj)
		self.publishPlannerParams(planner_YAML_obj)
		# wait 5 sec to establish global_planner node
		rospy.wait_for_service('global_planner'+'/'+str(self.node_name)+'/make_plan', 5)

	def plannPath(start,finish):
		make_plan_service = rospy.ServiceProxy('global_planner'+'/'+str(self.node_name)+'/make_plan', MakePlan)
		path = make_plan_service(start,finish)
		return path
	def setPlanningAlgorithm(algorithm):
		rospack = rospkg.RosPack()
		planning_data_path = rospack.get_path('path_planning_data')
		planner_YAML_path = planning_data_path+'/plannningYAML/'+str(algorithm)+'.yaml'
		return planner_YAML_path
	def setCostmapConfig(robot_type):
		rospack = rospkg.RosPack()
		planning_data_path = rospack.get_path('path_planning_data')
		robot_type_config = planning_data_path+'/costmapYAML/'+str(robot_type)+'.yaml'
		return robot_type_config
	def killNodes():
		self.process_GP.stop()
		self.process_tf.stop()
		self.process_MS.stop()



def signal_handler(signal, frame):
	print "[State server] - signal SIGINT caught"
	print "[State server] - system exits"
	
	sys.exit(0)

if __name__ == '__main__':
	try:
		path_planner = PathPlanning()
		algorithm_config = path_planner.setPlanningAlgorithm(req.algorithm)
		robot_type_config = path_planner.setCostmapConfig(req.robot_type)
		path_planner.initializeGlobalPlanner(req.map_path, robot_type_config, algorithm_config)
		path = path_planner.plannPath(req.start, req.finish)
		path_planner.killNodes()
		return PathPlanningRosSrvResponse(path)	
	except (KeyboardInterrupt, SystemExit):
		print "[Rapp Path planner node] - SystemExit Exception caught"
		
		myBroker.shutdown()
	
		sys.exit(0)
		
	except Exception, ex:
		print "[Rapp Path planner node] - Exception caught %s" % str(ex)
	
		myBroker.shutdown()
	
		sys.exit(0)