#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing auxiliary packages \e[0m"
sudo apt-get install -y openssh-server
sudo apt-get install -y vim
sudo apt-get install -y git gitg

