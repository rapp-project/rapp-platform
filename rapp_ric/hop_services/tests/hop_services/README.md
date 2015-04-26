Contains Hop Services invocation tests.
Requires a hop client server to run.
Each test runs on a hop client as an indepentend service.
Cannot be used with nodejs!!

---------------------------------------------------------
Current test implementations:
- faceDetection. [Calls faceDetection hop service]-->[Call faceDetection ROS service]-->[Return message to the client].
- qrDetection.[Calls fqretection hop service]-->[Call qrDetection ROS service]-->[Return message to the client].
- speech2text.[Calls speech2text hop service]-->[Call speech2text ROS service]-->[Return message to the client].
- storeFile. [Calls storeFile hop service]-->[Stores the file]-->[Return to client]. 

---------------------------------------------------------
Run exec.sh (./exec.sh) with 2 parameters:
- $1 : test to run (e.g faceDetection.js).
- $2 : Service listening port number (e.g. 9004)
- $3 : Number of calls.

Example of use:

``./exec.sh faceDetection.js 9002 10`` 
will execute faceDetection.js 10 times in series and the relevant hop executable is spawned at
portNumber:9002
