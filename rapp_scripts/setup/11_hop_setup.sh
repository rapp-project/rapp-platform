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
cd hop
git checkout 197bb542ad5e5a0ee1711a969e4a40880f9e3955
cd ../

tar -zxvf bigloo4.2a-beta16Sep15.tar.gz
cd bigloo4.2a
./configure
make
make test
make compile-bee
sudo make install
sudo make install-bee
cd ..
cd hop
./configure
make
sudo make install

# Initialize HOP with users
mkdir -p $HOME/.config/hop
cd $curr/hop_init
cp * $HOME/.config/hop/
