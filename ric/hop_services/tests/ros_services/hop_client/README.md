This Directory contains tests for calling a ROS service by invocing a hop client.

---------------------------
- Current implementation wokrs only under a hop client.
- Cannot be used with nodejs.

--------------------------
Run exec.sh (./exec.sh) with 2 parameters:
- $1 : test to run (e.g faceDetection.js).
- $2 : Number of calls.

Example of use:
./exec.sh faceDetection.js 10 will execute faceDetection.js 10 times in series.
