#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

#--------------------------------- Updates -----------------------------------#
bash ./1_system_updates.sh

#-------------------------------- ROS setup ----------------------------------#
bash ./2_ros_setup.sh

#------------------------ Auxiliary packages install -------------------------#
bash ./3_auxiliary_packages_setup.sh

#-------------------------------Github repos setup----------------------------#
bash ./4_rosjava_setup.sh
bash ./5_knowrob_setup.sh
bash ./6_rapp_platform_setup.sh
bash ./7_sphinx_libraries.sh

#-------------------------------MySQLsetup------------------------------------#
bash ./8_mysql_install.sh
bash ./9_create_rapp_mysql_db.sh
bash ./10_create_rapp_mysql_users.sh

#-------------------------------- HOP setup ----------------------------------#
bash ./11_hop_setup.sh

#-------------------------------- Finalizing ---------------------------------#
source ~/.bashrc
