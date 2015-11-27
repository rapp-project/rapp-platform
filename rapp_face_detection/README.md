Documentation about the RAPP Face detection: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Face-Detection)

#Methodology

In the RAPP case, the face detection functionality is implemented in the form of a C++ developed ROS node, interfaced by a HOP service. The HOP service is invoked using the RAPP API and gets an RGB image as input, in which faces must be detected. The second step is for the HOP service to locally save the input image. At the same time, the Face Detection ROS node is executed in the background, waiting to server requests. The HOP service calls the ROS service via the ROS Bridge, the ROS node make the necessary computations and a response is delivered.

#ROS Services

##Face detection 
Service URL: ```/rapp/rapp_face_detection/detect_faces```

Service type:
```bash
# Contains info about time and reference
Header header
# The image's filename to perform face detection
string imageFilename
---
# Container for detected face positions
geometry_msgs/PointStamped[] faces_up_left
geometry_msgs/PointStamped[] faces_down_right
string error
``` 

#Launchers

##Standard launcher

Launches the **face detection** node and can be launched using
```
roslaunch rapp_face_detection face_detection.launch
```

#HOP services

## URL
```localhost:9001/hop/face_detection ```

## Input / Output

```
Input = {
  “image”: “THE_ACTUAL_IMAGE_DATA”
}
```
```
Output = {
  “faces”: [
    {
      “top_left_x” : “T_P_X”,
      “top_left_y” : “T_P_Y”,
      “bottom_right_x” : “B_R_X”,
      “bottom_right_y” : “B_R_Y”
    },
    {
      …
    }
  ]
}
```
