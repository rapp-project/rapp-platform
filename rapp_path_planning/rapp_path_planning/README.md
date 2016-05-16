Documentation about the RAPP Path Planner: [link](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Path-Planner)

**Rapp_path_planner** <img src="https://farm6.staticflickr.com/5199/7369580478_aef5890b05_o_d.png" alt="Rapp_path_planner incon" align="right"  width="250" />
========



**Components**
----------

#### 1. *rapp_path_planning*


Rapp_path_planning is used in the RAPP case to plan path from given pose to given goal. User can costomize the path planning module with following parameters:
* prebuild map - avaliable maps are stored [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_map_server/maps),
* planning algorithm - **for now, only [dijkstra](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)  is avaliable**,
* robot type - customizes costmap for planning module. **For now only [NAO](https://www.aldebaran.com/en/humanoid-robot/nao-robot) is supported**. 

#### 2. *rapp_map_server*


Rapp_map_server delivers prebuild maps to rapp_path_planning component. All avaliable maps are contained [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_map_server/maps). 

This component is based on the ROS package: [map_server](http://wiki.ros.org/map_server). The rapp_map_server reads .png and .yaml files and publishes map as [OccupacyGrid](http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html) data. RAPP case needs run-time changing map publication, thus the rapp_map_server extands map_server functionality. The rapp_map_server enables user run-time changes of map. It subscribes to ROS parameter: 
```rospy.set_param(nodeName+"/setMap", map_path)```
and publishes the map specified in map_path. Examplary map changing request is presented below.
```python
nodename = rospy.get_name()
map_path = "/home/rapp/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_map_server/maps/empty.yaml"
rospy.set_param(nodename+/setMap, map_path)
```

A ROS service exists to store new maps in each user's workspace, called ```upload_map```. Then each application can invoke the ```planPath2D``` service, providing the map's name (among others) as input argument.

**ROS Services**
------------

#### *Path planning*


Service URL: ```/rapp/rapp_path_planning/planPath2D```

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

Service URL: ```/rapp/rapp_path_planning/upload_map```

Service type:
```bash
# The end user's username, since the uploaded map is personal 
string user_name
# The map's name. Must be unique for this user
string map_name
# The map's resolution
float32 resolution
# ROS-specific: The map's origin
float32[] origin
# ROS-specific: Whether the occupied / unoccupied pixels must be negated
int16 negate
# Occupied threshold
float32 occupied_thresh
# Unoccupied threshold
float32 free_thresh
# File size for sanity checks
uint32 file_size
# The map data
char[] data
---
byte status
``` 
More information on the Occupancy Grid Map representation can be found [here](http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html)

**Launchers**
-------------

#### *Standard launcher*


Launches the **path planning** node and can be launched using
```bash
roslaunch rapp_path_planning path_planning.launch
```


#Web services

## Path planning 2D

### URL
```localhost:9001/hop/path_planning_path_2d ```

### Input / Output

```
Input = {
  "map_name": “THE_PRESTORED_MAP_NAME”,
  "robot_type": "Nao",
  "algorithm": "dijkstra",
  "start": {x: 0, y: 10},
  "goal": {x: 10, y: 0}
}
```
```
Output = {
  "plan_found": 0,
  "path": [{x: 0, y: 10}, {x: ... ],
  "error": ""
}
```

## Upload map

### URL
```localhost:9001/hop/path_planning_upload_map ```

### Input / Output

```
Input = {
  "png_file": “map.png”,
  "yaml_file": "map.yaml",
  "map_name": "simple_map_1"
}
```
```
Output = {
  "error": ""
}
```

The full documentation exists [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_web_services/services#path-planning-plan-path-2d) and [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_web_services/services#path-planning-upload-map).


#### Author

[Wojciech Dudek](https://github.com/dudekw)

