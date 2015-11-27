Documentation about the RAPP MySQL Wrapper: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-MySQL-wrapper)

#Methodology

The RAPP MySQL wrapper ROS node provides the means to interact with the developed MySQL database by providing 26 ROS services that aid in selecting, inserting, updating or deleting information from some of the tables of the RAPP MySQL Database.

#ROS Services

A write, fetch, update and delete service exists for each of the tables of the MySQL database that are accessible through the wrapper. These four services follow a general approach, irrespective of the actual table they affect.
The URLs of the available services are shown below.

```
/rapp/rapp_mysql_wrapper/tbl_apps_robots_delete_data
/rapp/rapp_mysql_wrapper/tbl_apps_robots_fetch_data
/rapp/rapp_mysql_wrapper/tbl_apps_robots_update_data
/rapp/rapp_mysql_wrapper/tbl_apps_robots_write_data
/rapp/rapp_mysql_wrapper/tbl_model_delete_data
/rapp/rapp_mysql_wrapper/tbl_model_fetch_data
/rapp/rapp_mysql_wrapper/tbl_model_update_data
/rapp/rapp_mysql_wrapper/tbl_model_write_data
/rapp/rapp_mysql_wrapper/tbl_rapp_delete_data
/rapp/rapp_mysql_wrapper/tbl_rapp_fetch_data
/rapp/rapp_mysql_wrapper/tbl_rapp_update_data
/rapp/rapp_mysql_wrapper/tbl_rapp_write_data
/rapp/rapp_mysql_wrapper/tbl_robot_delete_data
/rapp/rapp_mysql_wrapper/tbl_robot_fetch_data
/rapp/rapp_mysql_wrapper/tbl_robot_update_data
/rapp/rapp_mysql_wrapper/tbl_robot_write_data
/rapp/rapp_mysql_wrapper/tbl_user_delete_data
/rapp/rapp_mysql_wrapper/tbl_user_fetch_data
/rapp/rapp_mysql_wrapper/tbl_user_update_data
/rapp/rapp_mysql_wrapper/tbl_user_write_data
/rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_delete_data
/rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_fetch_data
/rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_update_data
/rapp/rapp_mysql_wrapper/tbl_users_ontology_instances_write_data
/rapp/rapp_mysql_wrapper/view_users_robots_apps_fetch_data
/rapp/rapp_mysql_wrapper/what_rapps_can_run
```
Below is a detailed explanation of how the fetch, write, update and delete services work in general. Examples of how these services are called can be found in:
[https://github.com/rapp-project/rapp-platform/blob/master/rapp_mysql_wrapper/test/mysql_wrapper/db_test.py#L217-L304](https://github.com/rapp-project/rapp-platform/blob/master/rapp_mysql_wrapper/test/mysql_wrapper/db_test.py#L217-L304)

##Fetch Data
The general format of the service responsible for fetching data from an array is presented below.

```bash
# the requested column names in the MySQL select query
string[] req_cols
# a 2 dimensional array each line of which is a where relationship in the select query
rapp_platform_ros_communications/StringArrayMsg[] where_data
---
# the resulting column names
string[] res_cols
#the resulting column data (in 2 dimensional array) where each line is an entry
rapp_platform_ros_communications/StringArrayMsg[] res_data
# possible error information
string[] trace
# true if successful
std_msgs/Bool success
``` 

##Write Data
The general format of the service responsible for writing data to an array is presented below.

```bash
# the to be written column names
string[] req_cols
# the values to be written to the above columns
rapp_platform_ros_communications/StringArrayMsg[] req_data
---
# possible error information
string[] trace
# true if successful
std_msgs/Bool success
``` 

##Update Data
The general format of the service responsible for updating data of an array is presented below.

```bash
# the to be set columns along with their new values in 'columnName=newValue' format
string[] set_cols
# a 2 dimensional array each line of which is a where relationship
rapp_platform_ros_communications/StringArrayMsg[] where_data
---
# possible error information
string[] trace
# true if successful
std_msgs/Bool success
``` 

##Delete Data 
The general format of the service responsible for deleting data of an array is presented below.

```bash
# a 2 dimensional array each line of which is a where relationship
rapp_platform_ros_communications/StringArrayMsg[] where_data
---
# possible error information
string[] trace
# true if successful
std_msgs/Bool success
``` 

##whatRappsCanRun
This service returns what RApps can run on a robot when given the robot's model ID and it's Core Agent version.

```bash
# the robot's model id
string model_id
# the robot's core agent version
string core_agent_version
---
# the resulting column names
string[] res_cols
#the resulting column data (in 2 dimensional array) where each line is an entry
rapp_platform_ros_communications/StringArrayMsg[] res_data
# possible error information
string[] trace
# true if successful
std_msgs/Bool success
``` 
