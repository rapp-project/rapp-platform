Documentation about the RAPP MySQL Wrapper: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-MySQL-wrapper)

#Methodology

The RAPP MySQL wrapper ROS node provides the means to interact with the developed MySQL database by providing a number of ROS services that perform specific, predefined procedures of reading from and writing to the database.


#ROS Services
Each service is analyzed below.

## Add store token to device service

Service URL: ```/rapp/rapp_mysql_wrapper/add_store_token_to_device```

Service type:
```bash
string store_token
---
string error
```

## Check active application token service

Service URL: ```/rapp/rapp_mysql_wrapper/check_active_application_token```

Service type:
```bash
string application_token
---
bool application_token_exists
bool success
string error
string[] trace
```

## Check active robot session service

Service URL: ```/rapp/rapp_mysql_wrapper/check_active_robot_session```

Service type:
```bash
string username
string device_token
---
bool application_token_exists
bool success
string error
string[] trace
```

## Check if user exists service

Service URL: ```/rapp/rapp_mysql_wrapper/check_if_user_exists```

Service type:
```bash
string username
---
bool user_exists
bool success
string error
string[] trace
```

## Create new application token service

Service URL: ```/rapp/rapp_mysql_wrapper/create_new_application_token```

Service type:
```bash
string username
string store_token
string application_token
---
bool success
string error
string[] trace
```

## Create new cloud agent service

Service URL: ```/rapp/rapp_mysql_wrapper/create_new_cloud_agent```

Service type:
```bash
string username
string tarball_path
string container_identifier
string image_identifier
string container_type
---
bool success
string error
string[] trace
```

## Create new cloud agent service service

Service URL: ```/rapp/rapp_mysql_wrapper/create_new_cloud_agent_service```

Service type:
```bash
string container_identifier
string service_name
string service_type
int16 container_port
int16 host_port
---
bool success
string error
string[] trace
```

## Create new platform user service

Service URL: ```/rapp/rapp_mysql_wrapper/create_new_platform_user```

Service type:
```bash
string username
string password
string creator_username
string language
---
bool success
string error
string[] trace
```

## Get cloud agent service type and host port service

Service URL: ```/rapp/rapp_mysql_wrapper/get_cloud_agent_service_type_and_host_port```

Service type:
```bash
string container_identifier
string service_name
---
string service_type
int16 host_port
bool success
string error
string[] trace
```

## Get user language service

Service URL: ```/rapp/rapp_mysql_wrapper/get_user_language```

Service type:
```bash
string username
---
string user_language
bool success
string error
string[] trace
```

##Get username associated with application token service

Service URL: ```/rapp/rapp_mysql_wrapper/get_username_associated_with_application_token```

Service type:
```bash
string application_token
---
string username
bool success
string error
string[] trace
```

## Get user ontology alias service

Service URL: ```/rapp/rapp_mysql_wrapper/get_user_ontology_alias```

Service type:
```bash
string username
---
string ontology_alias
bool success
string error
string[] trace
```

## Get user password service

Service URL: ```/rapp/rapp_mysql_wrapper/get_user_password```

Service type:
```bash
string username
---
string password
bool success
string error
string[] trace
```

## Register user ontology alias service

Service URL: ```/rapp/rapp_mysql_wrapper/register_user_ontology_alias```

Service type:
```bash
string username
string ontology_alias
---
bool success
string error
string[] trace
```

## Remove platform user service

Service URL: ```/rapp/rapp_mysql_wrapper/remove_platform_user```

Service type:
```bash
string username
---
string password
bool success
string error
string[] trace
```

## Validate existing platform device token service

Service URL: ```/rapp/rapp_mysql_wrapper/validate_existing_platform_device_token```

Service type:
```bash
string device_token
---
bool device_token_exists
bool success
string error
string[] trace
```

## Validate user role service

Service URL: ```/rapp/rapp_mysql_wrapper/validate_user_role```

Service type:
```bash
string username
---
string error
string[] trace
```

# Launchers

## Standard Launcher
Launches the rapp_mysql_wrapper node and can be invoked using
```roslaunch rapp_mysql_wrapper mysql_wrapper.launch```

