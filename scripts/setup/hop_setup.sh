#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP by deb \e[0m"

sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" >> \
  /etc/apt/sources.list'
  
#sudo apt-get update
#sudo apt-get install -y hop
cd ~
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
sudo make install-bee
cd ..
cd hop-3.0.0-pre14
./configure
make
sudo make install
