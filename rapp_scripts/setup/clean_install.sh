#!/bin/bash

##
# MIT License (MIT)

# Copyright (c) <2014> <Rapp Project EU>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Authors: Manos Tsardoulias
# Contact: etsardou@iti.gr
##


# This script installs all the prerequisites and sets up all the needed
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

#--------------------------------- Updates -----------------------------------#
bash ./1_system_updates.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on system_updates";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------- ROS setup ----------------------------------#
bash ./2_ros_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing ROS";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#------------------------ Auxiliary packages install -------------------------#
bash ./3_auxiliary_packages_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing auxiliary_packages";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------Github repos setup----------------------------#
bash ./4_rosjava_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing rosjava";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

bash ./5_knowrob_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on knowrob setup";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

bash ./6_rapp_platform_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on rapp-platform setup";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

bash ./7_sphinx_libraries.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on sphinx-libs setup";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------MySQLsetup------------------------------------#
bash ./8_mysql_install.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing mysql";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

sudo bash ./9_create_rapp_mysql_db.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on creating Rapp-db";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc

bash ./10_create_rapp_mysql_users.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on creating db-users";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
#-------------------------------- HOP setup ----------------------------------#
bash ./11_hop_setup.sh || \
  {
    echo -e "[Error]: RAPP Platform installation failed on installing HOP";
    exit 1;
  }
sudo ldconfig
source ~/.bashrc
