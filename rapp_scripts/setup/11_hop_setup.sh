#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP by repos \e[0m"

# Keep current directory
curr=$(pwd)
#sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" >> \
#  /etc/apt/sources.list'

sudo apt-get install libunistring-dev

cd ~/rapp_platform
mkdir hop
cd hop
wget ftp://ftp-sop.inria.fr/indes/fp/Bigloo/bigloo4.2a-beta16Sep15.tar.gz
git clone https://github.com/manuel-serrano/hop.git
tar -zxvf bigloo4.2a-beta16Sep15.tar.gz
cd bigloo4.2a
./configure
make
make test
make compile-bee
sudo make install
sudo make -y install-bee
cd ..
cd hop
./configure
make
sudo make install

# Initialize HOP with users
mkdir -p $HOME/.config/hop
cd $curr/hop_init
cp * $HOME/.config/hop/
