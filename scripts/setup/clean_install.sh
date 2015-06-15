#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

#--------------------------------- Updates -----------------------------------#
bash ./system_updates.sh #ok
#source ~/.bashrc
#sudo ldconfig
#-------------------------------- ROS setup ----------------------------------#
bash ./ros_setup.sh #ok
#source ~/.bashrc
#sudo ldconfig
#------------------------ Auxiliary packages install -------------------------#
bash ./auxiliary_packages_setup.sh #ok
#bash ./rosjava.sh
#source ~/.bashrc
#sudo ldconfig
#bash ./knowrob.sh
#-------------------------------- HOP setup ----------------------------------#
#bash ./hop_setup.sh
#-------------------------------Github repos setup----------------------------#
bash ./external_repos_setup.sh
#source ~/.bashrc
#sudo ldconfig
#bash ./rosjava.sh
#source ~/.bashrc
#sudo ldconfig
#bash ./knowrob.sh
bash ./rapp_platform_setup.sh
bash ./sphinx_libraries.sh
bash ./mysql_setup.sh
bash ./hop_setup.sh
