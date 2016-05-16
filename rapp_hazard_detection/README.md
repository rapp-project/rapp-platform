Documentation about the RAPP Hazard Detection: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Hazard-Detection)

#Methodology

In the RAPP case, the hazard detection functionality is implemented in the form of a C++ developed ROS node, interfaced by a Web service. The Web service is invoked using the RAPP API and gets an RGB image as input, in which hazards has to be checked. The second step is for the Web service to locally save the input image. At the same time, the hazard_detection ROS node is executed in the background, waiting to server requests. The Web service calls the ROS service via the ROS Bridge, the ROS node make the necessary computations and a response is delivered.

#ROS Services

##Light checking
Service URL: ```/rapp/rapp_hazard_detection/light_check```

Service type:
```bash
# Contains info about time and reference
Header header
# The image's filename to perform light checking
string imageFilename
---
# Light level in the center of the provided image
int32 light_level
string error
``` 

##Door checking
Service URL: ```/rapp/rapp_hazard_detection/light_check```

Service type:
```bash
# Contains info about time and reference
Header header
# The image's filename to perform door checking
string imageFilename
---
# Estimated door opening angle
int32 door_angle
string error
``` 

#Launchers

##Standard launcher

Launches the **hazard_detection** node and can be launched using
```
roslaunch rapp_hazard_detection hazard_detection.launch
```

#Web services

## Light checking

### URL
```localhost:9001/hop/hazard_detection_light_check ```

### Input / Output

```
Input = {
  "file": “THE_ACTUAL_IMAGE_DATA”
}
```
```
Output = {
  "light_level": 50,
  "error": ""
}
```

## Door checking

### URL
```localhost:9001/hop/hazard_detection_door_check ```

### Input / Output

```
Input = {
  "file": "THE_ACTUAL_IMAGE_DATA"
}
```
```
Output = {
  "door_angle": 50,
  "error": ""
}
```

The full documentation exists [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_web_services/services#hazard-detection-door-check) and [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_web_services/services#hazard-detection-door-check)
