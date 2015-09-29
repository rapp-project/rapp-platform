#Setup scripts

These scripts can be executed after a clean Ubuntu 14.04 installation, in order
to install the appropriate packages and setup the environment.

##Step 0 - Github keys setup

Since our Github repositories are private, a public key to Github should be 
set in order to be able to clone the repositories. This can be performed by 
following the instructions:

https://help.github.com/articles/generating-ssh-keys/

If you already have a public/private key, copy it in the ```.ssh``` folder
of your new installation.

Additionally install git in order to be able to clone the repositories:

```sudo apt-get install git```

WARNING: At least 10 GB's of free space are recommended.

##Updates / ROS install / Github repos
--------------------------------------------
Execute clean_install.sh script

Performs:
- initial system updates 
- installs ROS Indigo 
- downloads all Github repos needed
- builds and install all repos (rapp_platform, knowrob, rosjava)
- downloads builds and installs depending libraries for Sphinx4
- installs MySQL
- installs HOP


##Note: What you must do manually

A new MySQL user was created with username = 'dummyUser' and password = 'changeMe' and was granted all on RappStore DB. It is highly recommended that you change the password and the username of the user. The username and password are stored in the file located at /etc/db_credentials. The file db_credentials is used by the rapp platform services, be sure to update it with the correct username and password. It's first line is the username and it's second line the password.

##Setup in an existing system

If you want to add rapp-platform to an already existent system (Ubuntu 14.04) you must:

- Create a catkin workspace (if you dont have one)
 - ```mkdir PATH/rapp-platform-catkin-ws```
 - ```cd PATH/rapp-platform-catkin-ws```
 - ```mkdir src && cd src```
 - ```catkin_init_workspace```
- Then clone the rapp-platform workspace in the ```src``` folder, as it is constructed as a ROS metapackage.
- To compile it just ```cd /PATH/rapp-platform-catkin-ws && catkin_make```

#####NOTES:

The following notes concern the manual setup of rapp-platform (not the clean setup form our scripts):

- To compile ```rapp_qr_detection``` you must install the ```libzbar``` library
- To compile ```rapp_knowrob_wrapper``` you must execute the following scripts:
 - https://github.com/rapp-project/rapp-platform/blob/master/rapp_scripts/setup/4_rosjava_setup.sh
 - https://github.com/rapp-project/rapp-platform/blob/master/rapp_scripts/setup/5_knowrob_setup.sh
 - **If you dont want interaction with the ontology, add an empty ```CATKIN_IGNORE``` file in the ```rapp-platform/rapp_knowrob_wrapper/``` folder**
