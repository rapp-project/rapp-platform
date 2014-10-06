#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

# Step 2 : Github/ ROS repositories initialization

# Here you must add your public key to github for the clone command to be
# be able to execute, as the repos are private

#---------------------- RAPP Github repositories setup -----------------------#

# Create folder for RAPP platform repo
cd ~/Desktop
mkdir rapp-repos && cd rapp-repos
# Clone the repository (public key should have been setup)
git clone git@github.com:rapp-project/rapp-platform.git

#-------------------------------- HOP setup ----------------------------------#


