#Instructions

Currently the server works on localhost:9001. To test it you must:

- ```roslaunch rapp_server rapp_server.launch```
- ```roslaunch ros_nodes face_detection.launch```
- Go to the ```rapp-api/cpp```, build it and execute:
 - ```./face_detect PATH_TO_IMAGE.png```

TODO: 
- Take IP, port as rosparam from the launch file
- Check image type and save it accordingly
