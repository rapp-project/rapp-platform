#rapp-platform
Contains the RAPP Store, Platform, Server and RIC

##Setup
To setup rapp-platform from scratch it is advised to follow the intructions here [still under construction]:

https://github.com/rapp-project/rapp-platform/tree/master/scripts/setup

If you want to add rapp-platform to an already existent system (Ubuntu 14.04) you must:

- Create a catkin workspace (if you dont have one)
 - ```mkdir PATH/rapp_ws```
 - ```cd PATH/rapp_ws```
 - ```mkdir src && cd src```
 - ```catkin_init_workspace```
- Then clone the rapp-platform workspace in the ```src``` folder, as it is constructed as a ROS metapackage.
- To compile it just ```cd /PATH/rapp_ws && catkin_make```

#####NOTES:

- To compile ```RIC / ros_nodes``` you must install the ```libzbar``` library
- To compile ```RIC / ros_wrappers``` you must setup the following catkin_workspaces:
 - ```https://github.com/rosjava/rosjava```
 - ```https://github.com/knowrob/knowrob/tree/indigo-devel```
 - If you dont want interaction with the ontology, add an empty ```CATKIN_IGNORE``` file in the ```rapp-platform/rapp_ric/rapp_knowrob_wrapper/``` folder

