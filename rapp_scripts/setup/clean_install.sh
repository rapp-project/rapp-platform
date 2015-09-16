#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

#--------------------------------- Updates -----------------------------------#
bash ./1_system_updates.sh
sudo ldconfig
source ~/.bashrc
#-------------------------------- ROS setup ----------------------------------#
bash ./2_ros_setup.sh
sudo ldconfig
source ~/.bashrc
#------------------------ Auxiliary packages install -------------------------#
bash ./3_auxiliary_packages_setup.sh
sudo ldconfig
source ~/.bashrc
#-------------------------------Github repos setup----------------------------#
bash ./4_rosjava_setup.sh
sudo ldconfig
source ~/.bashrc

bash ./5_knowrob_setup.sh
sudo ldconfig
source ~/.bashrc

bash ./6_rapp_platform_setup.sh
sudo ldconfig
source ~/.bashrc

bash ./7_sphinx_libraries.sh
sudo ldconfig
source ~/.bashrc
#-------------------------------MySQLsetup------------------------------------#
bash ./8_mysql_install.sh
sudo ldconfig
source ~/.bashrc

sudo bash ./9_create_rapp_mysql_db.sh
sudo ldconfig
source ~/.bashrc

bash ./10_create_rapp_mysql_users.sh
sudo ldconfig
source ~/.bashrc
#-------------------------------- HOP setup ----------------------------------#
bash ./11_hop_setup.sh
sudo ldconfig
source ~/.bashrc
