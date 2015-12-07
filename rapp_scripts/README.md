Documentation about the RAPP Scripts: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Scripts)

The ```rapp_scripts``` folder contains scripts necessary for operations related to the RAPP Platform. The scripts are divided into four folders:

## backup

The following scripts are offered:
- ```dumpRAPPMysqlDatabase.sh```: Dumps the RAPP MySQL database in a .sql file
- ```importRAPPMysqlDatabase.sh```: Imports the RAPP MySQL database from a .sql file
- ```dumpRAPPOntology.sh```: Dumps the RAPP ontology in an .owl file
- ```importRAPPOntology.sh```: Imports the RAPP ontology from an .owl file

## continuous_integration

This folder contains the Travis script, used to support continuous integration of the RAPP Platform. The RAPP Platform Travis page is located [here](https://travis-ci.org/rapp-project/rapp-platform).
For every push performed in rapp-platform Travis:
- Checks the clean_install script
- Builds the rapp-platform repository
- Runs the unit, functional and integration tests
- Creates the code documentation and uploads it [online](http://rapp-project.github.io/rapp-platform/documentation.html)

## deploy

There are two files aimed for deployment:

- ```deploy_rapp_ros.sh```: Deploys the RAPP Platform back-end, i.e. all the ROS nodes
- ```deploy_hop_services.sh```: Deploys the corresponding HOP services

If you want to deploy the RAPP Platform in the background you can use ```screen```. Just follow the next steps:

- ```screen```
- ```./deploy_rapp_ros.sh```
- Press Ctrl + a + d to detach
- ```screen```
- ```./deploy_hop_services```
- Press Ctrl + a + d to detach
- ```screen -ls``` to check that 2 screen sessions exist

To reattach to screen session:
```
screen -r [pid.]tty.host
```
The screen step is for running rapp_ros and hop_services on detached terminals which is useful, for example in the case where you want to connect via ssh to a remote computer, launch the processes and keep them running even after closing the connection. Alternatively, you can open two terminals and run one script on each, without including the screen commands. It is imperative for the terminals to remain open for the processes to remain active.

Screen how-to: http://www.rackaid.com/blog/linux-screen-tutorial-and-how-to/

## setup

These scripts can be executed after a clean Ubuntu 14.04 installation, in order
to install the appropriate packages and setup the environment.

#### Step 0 - Get the scripts

You can get the setup scripts either by downloading the rapp-platform repository in a [zip format](https://github.com/rapp-project/rapp-platform/zipball/master), or by cloning it in your PC using git:

```git clone https://github.com/rapp-project/rapp-platform.git```

WARNING: At least 10 GB's of free space are recommended.

#### Install RAPP Platform

It is advised to execute the clean_install.sh script in a clean VM or clean system.

Performs:
- initial system updates 
- installs ROS Indigo 
- downloads all Github repositories needed
- builds and install all repos (rapp_platform, knowrob, rosjava)
- downloads builds and installs depending libraries for Sphinx4
- installs MySQL
- installs HOP

#### What you must do manually

A new MySQL user was created with username = ```dummyUser``` and password = ```changeMe``` and was granted all on RappStore DB. It is highly recommended that you change the password and the username of the user. The username and password are stored in the file located at /etc/db_credentials. The file db_credentials is used by the RAPP platform services, be sure to update it with the correct username and password. It's first line is the username and it's second line the password.

#### Setup in an existing system

If you want to add rapp-platform to an already existent system (Ubuntu 14.04) you should choose which setup scripts you need to execute. For example if you have MySQL install you do not need to execute ```8_mysql_install.sh```.

#### Scripts

- ```1_system_updates.sh```: Fetches the Ubuntu 14.04 updates and installs them
- ```2_ros_setup.sh```: Installs ROS Indigo
- ```3_auxiliary_packages_setup.sh```: Installs software from apt-get, necessary for the correct RAPP Platform deployment
- ```4_rosjava_setup.sh```: Fetches a number of GitHub repositories and compiles rosjava. This is a dependency of Knowrob.
- ```5_knowrob_setup.sh```: Fetches the Knowrob repository and builds it. Knowrob is the tool that deploys the RAPP ontology.
- ```6_rapp_platform_setup.sh```: Fetches the rapp-platform and rapp-api repositories and builds them. This script has an input argument which is the git branch the rapp-platform will be checked out. If you decide to execute this script manually it is recommended to give ```master``` as input.
- ```7_sphinx_libraries.sh```: Fetches the Sphinx4 necessary libraries, compiles them and installs them, Sphinx4 is used for ASR (Automatic Speech Recognition).
- ```8_mysql_install.sh```: Installs MySQL
- ```9_create_rapp_mysql_db.sh```: Adds the RAPP MySQL empty database in mysql
- ```10_create_rapp_mysql_user.sh```: Creates a user to enable access the to database from the RAPP Platform code
- ```11_hop_setup.sh```: Installs HOP and Bigloo, the tools providing the RAPP Platform generic services.

##### NOTES:

The following notes concern the manual setup of rapp-platform (not the clean setup form our scripts):

- To compile ```rapp_qr_detection``` you must install the ```libzbar``` library
- To compile ```rapp_knowrob_wrapper``` you must execute the following scripts:
 - https://github.com/rapp-project/rapp-platform/blob/master/rapp_scripts/setup/4_rosjava_setup.sh
 - https://github.com/rapp-project/rapp-platform/blob/master/rapp_scripts/setup/5_knowrob_setup.sh
 - **If you don't want interaction with the ontology, add an empty ```CATKIN_IGNORE``` file in the ```rapp-platform/rapp_knowrob_wrapper/``` folder**

## documentation

Scripts to automatically create documentation from the RAPP Platform code, the services and this wiki, using Doxygen.
All documents can be bound in ```${HOME}/rapp_platform_files/documentation```.

- `create_source_documentation.sh` : Creates the platform's source code documentation (cpp, h, py, java).
- `create_wiki_documentation.sh` : Creates the platform's GitHub Wiki documentation.
- `create_test_documentation.py` : Creates the platform's test source code documentation.
- `create_web_services_documentation.sh` : Creates the rapp_web_services documentation.
- `create_documentation.sh` : Creates all aforementioned documentations.
- `update_rapp-project.github.io.sh` : Creates the documentation and pushes it in the ```gh-pages``` branch of rapp-platform, in order to update the online pages. NOTE: This is functional only when executed in the Travis environment, so do not try to run it!

## devel

Use the [create_rapp_user.sh](https://github.com/rapp-project/rapp-platform/blob/master/rapp_scripts/devel/create_rapp_user.sh) to create and authenticate a new RAPP User.

The script is located under the [devel](https://github.com/rapp-project/rapp-platform/tree/master/rapp_scripts/devel) directory of the [rapp_scripts](https://github.com/rapp-project/rapp-platform/tree/master/rapp_scripts) package.

```shell
$ cd ~/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_scripts/devel
$ ./create_rapp_user.sh
```

The script will prompt to input required info

```shell
$ ./create_rapp_user.sh

Minimal required fields for mysql user creation:
* username  : 
* firstname : User's firstname
* lastname  : User's lastname/surname
* language  : User's first language

Username: rapp
Firstname: rapp
Lastname: rapp
Language: el
Password: 
```
