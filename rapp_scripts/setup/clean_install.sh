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
RAPP_PLATFORM_BRANCH='master'
if [ -n "${TRAVIS_BRANCH}" ]; then
  RAPP_PLATFORM_BRANCH="${TRAVIS_BRANCH}"
fi
./6_rapp_platform_setup.sh ${RAPP_PLATFORM_BRANCH} || \
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
# Travis Related parameters handling
SQL_PARAM=''
if [ $# -ne 0 ]; then
  if [ $1 == 'travis' ]; then
    SQL_PARAM='travis'
  fi
fi

./8_mysql_install.sh ${SQL_PARAM} || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing mysql";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

bash ./9_create_rapp_mysql_db.sh ${SQL_PARAM} || \
  {
    echo -e "[Error]: RAPP Platform installation failed on creating Rapp-db";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

./10_create_rapp_mysql_users.sh ${SQL_PARAM} || \
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
#-------------------------------- Caffe setup ----------------------------------#
./12_caffe_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing Caffe";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#--------------------------- Authentication Setup ----------------------------#
./13_authentication_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on authentication setup";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

