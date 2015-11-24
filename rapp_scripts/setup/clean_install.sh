#!/bin/bash -i

##

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Authors: Manos Tsardoulias
# Contact: etsardou@iti.gr
##


# This script installs all the prerequisites and sets up all the needed
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

#--------------------------------- Updates -----------------------------------#
./1_system_updates.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on system_updates";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------- ROS setup ----------------------------------#
./2_ros_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing ROS";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#------------------------ Auxiliary packages install -------------------------#
./3_auxiliary_packages_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing auxiliary_packages";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------Github repos setup----------------------------#
./4_rosjava_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing rosjava";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

./5_knowrob_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on knowrob setup";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

./6_rapp_platform_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on rapp-platform setup";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

./7_sphinx_libraries.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on sphinx-libs setup";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------MySQLsetup------------------------------------#
./8_mysql_install.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing mysql";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

bash ./9_create_rapp_mysql_db.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on creating Rapp-db";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

./10_create_rapp_mysql_users.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on creating db-users";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------- HOP setup ----------------------------------#
./11_hop_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing HOP";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
