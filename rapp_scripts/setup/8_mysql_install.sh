#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] MySQL install \e[0m"

# Setup sources list
sudo apt-get -y install mysql-client mysql-server
sudo apt-get -y install python-mysqldb

