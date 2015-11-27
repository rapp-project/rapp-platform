Documentation about the RAPP Platform ROS Communications: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Platform-ROS-Communications)

The ```rapp_platform_ros_communications``` ROS package contains all the necessary ROS message, service and action files the RAPP Platform nodes require to operate. These are placed in the respoctive ```msg```, ```srv``` and ```action``` folders. In these folders, each file is being kept under a dedicated folder for each ROS package.

This way, each ROS node has to declare the ```rapp_platform_ros_communications``` as dependency, in order to use ROS messages, services or actions.
