Documentation about the RAPP Path Planner: [link](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Path-Planner)

**Rapp_path_planner** <img src="https://farm6.staticflickr.com/5199/7369580478_aef5890b05_o_d.png" alt="Rapp_path_planner incon" align="right"  width="250" />
========



**Components**
----------

rapp_path_planning
------------------

Rapp_path_planning is used in the RAPP case to plan path from given pose to given goal. User can costomize the path planning module with following parameters:
* pebuild map - avaliable maps are stored [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_map_server/maps),
* planning algorithm - **for now, only [dijkstra](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)  is avaliable**,
* robot type - customizes costmap for planning module. **For now only [NAO](https://www.aldebaran.com/en/humanoid-robot/nao-robot) is supported**. 

rapp_map_server
---------------

Rapp_map_server delivers prebuild maps to rapp_path_planning component. All avaliable maps are contained [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_map_server/maps). 

This component is based on the ROS package: [map_server](http://wiki.ros.org/map_server). The rapp_map_server reads .png and .yaml files and publishes map as [OccupacyGrid](http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html) data. RAPP case needs run-time changing map publication, thus the rapp_map_server extands map_server functionality. The rapp_map_server enables user run-time changes of map. It subscribes to ROS parameter: 
```rospy.set_param(nodeName+"/setMap", map_path)```
and publishes the map specified in map_path. Examplary map changing request is presented below.
```python
nodename = rospy.get_name()
map_path = "/home/rapp/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_map_server/maps/empty.yaml"
rospy.set_param(nodename+/setMap, map_path)
```


----------


**ROS Services**
------------

Path planning
-------------

Service URL: ```/rapp/rapp_path_planning/plan_path```

Service type:
```bash
# Contains name to the desired map
string map_name
# Contains type of the robot. It is required to determine it's parameters (footprint etc.)
string robot_type
# Contains path planning algorithm name
string algorithm
# Contains start pose of the robot
geometry_msgs/PoseStamped start
# Contains goal pose of the robot
geometry_msgs/PoseStamped goal
---
# status of the service
# plan_found:
#          * 0 : path cannot be planned.
#          * 1 : path found 
#          * 2 : wrong map name
#          * 3 : wrong robot type
#          * 4 : wrong algorithm


uint8 plan_found
# error_message : error explanation
string error_message
# path : vector of PoseStamped objects
# if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal
geometry_msgs/PoseStamped[] path
``` 

**Launchers**
-------------

Standard launcher
-----------------

Launches the **path planning** node and can be launched using
```bash
roslaunch rapp_path_planning path_planning.launch
```


#### Author

[Wojciech Dudek](https://github.com/dudekw)

