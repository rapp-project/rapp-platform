Documentation about the RAPP MySQL Wrapper: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-MySQL-wrapper)

#Methodology

The RAPP MySQL wrapper ROS node provides the means to interact with the developed MySQL database by providing a number of ROS services that perform specific, predefined procedures of reading from and writing to the database.


#ROS Services
The URLs of all the available services are shown below.

```
/rapp/rapp_mysql_wrapper/add_store_token_to_device
/rapp/rapp_mysql_wrapper/check_active_application_token
/rapp/rapp_mysql_wrapper/check_active_robot_session
/rapp/rapp_mysql_wrapper/check_if_user_exists
/rapp/rapp_mysql_wrapper/create_new_application_token
/rapp/rapp_mysql_wrapper/create_new_cloud_agent
/rapp/rapp_mysql_wrapper/create_new_cloud_agent_service
/rapp/rapp_mysql_wrapper/create_new_platform_user
/rapp/rapp_mysql_wrapper/get_cloud_agent_service_type_and_host_port
/rapp/rapp_mysql_wrapper/get_user_language
/rapp/rapp_mysql_wrapper/get_username_associated_with_application_token
/rapp/rapp_mysql_wrapper/get_user_ontology_alias
/rapp/rapp_mysql_wrapper/get_user_password
/rapp/rapp_mysql_wrapper/register_user_ontology_alias
/rapp/rapp_mysql_wrapper/remove_platform_user
/rapp/rapp_mysql_wrapper/validate_existing_platform_device_token
/rapp/rapp_mysql_wrapper/validate_user_role
```

Each service is analyzed in detail below.

## Add store token to device service

Service ULR: ```/rapp/rapp_mysql_wrapper/add_store_token_to_device```

Service type:
```bash
string store_token
---
string error
```

## Check active application token service

Service ULR: ```/rapp/rapp_mysql_wrapper/check_active_application_token```

Service type:
```bash
string application_token
---
bool application_token_exists
bool success
string error
string[] trace
```













# Launchers


# Web services
