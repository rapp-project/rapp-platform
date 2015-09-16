## Synopsis

This directory includes testing tools related to HOP Web Services located under the RAPP Platform, instructions to deploy the unit and functional tests, as well as the data needed.

### Unit and functional tests

In order to deploy the unit and functional tests do the following:

```
cd ~/rapp_platform/rapp-platform-catkin-ws/
catkin_make run_tests -j1
```

In order to deploy the tests of a single ROS package (e.g. face detection), perform the following:

```
cd ~/rapp_platform/rapp-platform-catkin-ws/
catkin_make run_tests_rapp_face_detection -j1
```

The ```rapp_face_detection``` postfix is the name of the corresponding ROS package's name.

### Integration testing

Follow the instructions here: https://github.com/rapp-project/rapp-platform/blob/master/rapp_testing_tools/testing_tools/README.md

### Directories

- testing_tools : Tools for testing HOP web services and furthermore RIC AI modules.

## Contributors

- Konstaninos Panayiotou, **[klpanagi@gmail.com]**
- Manos Tsardoulias, **[etsardou@gmail.com]**
