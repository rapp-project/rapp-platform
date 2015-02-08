#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP by deb \e[0m"

sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" >> \
  /etc/apt/sources.list'
  
sudo apt-get update
sudo apt-get install -y hop


