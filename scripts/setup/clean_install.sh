#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

#--------------------------------- Updates -----------------------------------#
bash ./system_updates.sh
#-------------------------------- ROS setup ----------------------------------#
bash ./ros_setup.sh
#------------------------ Auxiliary packages install -------------------------#
bash ./auxiliary_packages_setup.sh
#-------------------------------- HOP setup ----------------------------------#
bash ./hop_setup.sh
#-------------------------------Github repos setup----------------------------#
bash ./external_repos_setup.sh
bash ./rapp_platform_setup.sh

