Documentation about the RAPP Hazard Detection: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Hazard-Detection)

#Methodology

In the RAPP case, the hazard detection functionality is implemented in the form of a C++ developed ROS node, interfaced by a HOP service. The HOP service is invoked using the RAPP API and gets an RGB image as input, in which hazards has to be checked. The second step is for the HOP service to locally save the input image. At the same time, the hazard_detection ROS node is executed in the background, waiting to server requests. The HOP service calls the ROS service via the ROS Bridge, the ROS node make the necessary computations and a response is delivered.

#ROS Services

##Human detection
Service URL: ```/rapp/rapp_hazard_detection/light_check```

Service type:
```bash
# Contains info about time and reference
Header header
# The image's filename to perform light checking
string imageFilename
---
# List of bounding box borders, where the humans were detected 
geometry_msgs/PointStamped[] humans_up_left
geometry_msgs/PointStamped[] humans_down_right
# Possible error
string error
``` 

#Launchers

##Standard launcher

Launches the **human_detection** node and can be launched using
```
roslaunch rapp_human_detection human_detection.launch
```

#HOP services

## Human detection

### URL
```localhost:9001/hop/detect_humans ```

### Input / Output

```
Input = {
  "image": “THE_ACTUAL_IMAGE_DATA”
}
```
```
Output = {
  "human": { "up_left_point": {x: 0, y: 0}, "down_right_point": {x: 0, y: 0} }
}
```

