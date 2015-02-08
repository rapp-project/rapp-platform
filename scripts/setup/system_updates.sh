#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Initiating source lists \e[0m"
# Refresh source lists
sudo apt-get update

echo -e "\e[1m\e[103m\e[31m [RAPP] Upgrading packages \e[0m"
# Upgrades the current packages
sudo apt-get upgrade -y


