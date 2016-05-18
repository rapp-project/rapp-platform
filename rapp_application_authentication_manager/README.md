RAPP application authentication allows the various RAPP Platform users to
authenticate in order to gain access to the RAPP Platform services.

# ROS Services

## Add new user from Platform
Add new user using Platform credentials.

Service URL: ```/rapp/rapp_application_authentication/add_new_user_from_platfrom```

Service type:

AddNewUserFromPlatformSrv.srv
```
string creator_username
string creator_password
string new_user_username
string new_user_password
# New user's language
string language
---
string error
# Suggested username if provided already exists
string suggested_username
```

## Add new user from Store
Add new user using Store credentials.

Service URL: ```/rapp/rapp_application_authentication/add_new_user_from_store```

Service type:

AddNewUserFromStoreSrv.srv
```
# New user's username
string username
# New user's password
string password
# Creator device token from RAPP Store
string device_token
# New user's language
string language
---
string error
# Suggested username if provided already exists
string suggested_username
```


## Authenticate Token
Authenticates an active application token.

Service URL: ```/rapp/rapp_application_authentication/authenticate_token```

Service type:

UserTokenAuthenticationSrv.srv
```
string token
---
string error
# The token corresponding username
string username
```


## Login
Allows a user to login useing the RAPP Platform credentials.

Service URL: ```/rapp/rapp_application_authentication/login```

Service type:

UserLoginSrv.srv
```
string username
string password
# The device from which a user tries to login
string device_token
---
string error
string token
```


## Login from Store
Allows a user to login useing the RAPP Store token.

Service URL: ```/rapp/rapp_application_authentication/login_from_store```

Service type:

UserLoginSrv.srv
```
string username
string password
# The RAPP Store token
string device_token
---
string error
string token
```


# Launchers

## Standard launcher

Launches the **rapp application authentication** node and can be invoked using:
```bash
roslaunch rapp_application_authentication_manager application_authentication.launch
```

# Web Services

## Login User

Service URL: `localhost:9001/hop/login_user`

### Input/Output

```
Input = {
    username: '',
    password: '',
    device_token: ''
}
```
```
Output = {
    token: '',
    error: ''
}
```


## Register user from Platform

Service URL: `localhost:9001/hop/register_user_from_platform`

### Input/Output

```
Input = {
    creator_username: '',
    creator_password: '',
    new_user_username: '',
    new_user_password: '',
    language: ''
}
```
```
Output = {
    suggested_username: '',
    error: ''
}
```


## Register user from Store

Service URL: `localhost:9001/hop/register_user_from_store`

### Input/Output

```
Input = {
    username: '',
    password: '',
    device_token: '',
    language: ''
}
```
```
Output = {
    suggested_username: '',
    error: ''
}
```
