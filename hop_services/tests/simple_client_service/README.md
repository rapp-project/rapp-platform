This directory contains the following files:

1) listener.service.js --> Simple service which takes as input a string and outputs the input value to the console
Initiate the listener service on machine "A":
 ``` hop -v -g -p 9001 listener.service.js ```

2) talker.js --> javascript executable which invokes the above mentioned 'listener' service.
Run the talker javascript executable on machine "B":  
 ```hop -v -g -p 9001 talker.js ```

** Define the following variables, located int the talker.js script respectively: **
1) serverIP: The machines public IP, where the listener service is up and running
2) hopService_listeningPort: The port number which listener service was initiated
3) hopUser_username
4) hopUser_password 

Comments:
1) By defining the service with fixed parameters (service listener(say)), the invocation works as expected.
2) By defining the service with named parameters (service listener({say:''})), the error is produced!!!
3) The error appears on any of the above service definition, only when invoking the service from a remote machine!!!!



