#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP by repos \e[0m"

# Keep current directory
curr=$(pwd)
#sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" >> \
#  /etc/apt/sources.list'
  
cd ~/rapp_platform
mkdir hop
cd hop
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2015-05-07/bigloo4.2a.tar.gz
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2015-05-07/hop-3.0.0-pre14.tar.gz
tar -zxvf bigloo4.2a.tar.gz
tar -zxvf hop-3.0.0-pre14.tar.gz
cd bigloo4.2a
./configure
make
make test
make compile-bee
sudo make install
sudo make -y install-bee
cd ..
cd hop-3.0.0-pre14
./configure
make
sudo make install

# Initialize HOP with users
mkdir -p $HOME/.config/hop
cd $curr/hop_init
cp * $HOME/.config/hop/
