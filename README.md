#rapp-platform
Contains the RAPP Platform a.k.a. RIC (RAPP Improvement Center)

##Setup

To setup rapp-platform from scratch it is advised to follow the intructions here:

https://github.com/rapp-project/rapp-platform/tree/master/rapp_scripts/setup

If you want to add rapp-platform to an already existent system (Ubuntu 14.04) you must:

- Create a catkin workspace (if you dont have one)
 - ```mkdir PATH/rapp-platform-catkin-ws```
 - ```cd PATH/rapp-platform-catkin-ws```
 - ```mkdir src && cd src```
 - ```catkin_init_workspace```
- Then clone the rapp-platform workspace in the ```src``` folder, as it is constructed as a ROS metapackage.
- To compile it just ```cd /PATH/rapp-platform-catkin-ws && catkin_make```

##Testing

You can check that all the packages and procedures are correctly deployed using the instructions here:

https://github.com/rapp-project/rapp-platform/tree/master/rapp_testing_tools

#####NOTES:

The following notes concern the manual setup of rapp-platform (not the clean setup form our scripts):

- To compile ```rapp_qr_detection``` you must install the ```libzbar``` library
- To compile ```rapp_knowrob_wrapper``` you must execute the following scripts:
 - https://github.com/rapp-project/rapp-platform/blob/master/rapp_scripts/setup/4_rosjava_setup.sh
 - https://github.com/rapp-project/rapp-platform/blob/master/rapp_scripts/setup/5_knowrob_setup.sh
 - **If you dont want interaction with the ontology, add an empty ```CATKIN_IGNORE``` file in the ```rapp-platform/rapp_knowrob_wrapper/``` folder**

