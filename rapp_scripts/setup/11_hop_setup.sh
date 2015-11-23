#!/bin/bash

##
# MIT License (MIT)

# Copyright (c) <2014> <Rapp Project EU>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Authors: Konstantinos Panyiotou
# Contact: klpanagi@gmail.com
##

# Explicity check the result of each command and return with fail status if
# an error occures.
set -e


RappPlatformPath="${HOME}/rapp_platform"
BiglooIndex="bigloo4.2c-beta17Nov15"
BiglooUrl="ftp://ftp-sop.inria.fr/indes/fp/Bigloo/${BiglooIndex}.tar.gz"
HopRepoUrl="https://github.com/manuel-serrano/hop.git"
HopCommitIndex="7673c6e318425e880a05818626f689229699e157"

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP by repos \e[0m"

# Keep current directory
curr=$(pwd)
#sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" >> \
#  /etc/apt/sources.list'

sudo apt-get install libunistring-dev

# Bigloo installation
cd ${RappPlatformPath}
mkdir hop-bigloo
cd hop-bigloo
wget ${BiglooUrl}

tar -zxvf "${BiglooIndex}.tar.gz"
cd bigloo4.2c
./configure
make
make test
make compile-bee
sudo make install
sudo make install-bee

# HOP installation
cd ..
git clone ${HopRepoUrl}
cd hop
git checkout ${HopCommitIndex}
./configure
make
sudo make install
make doc
sudo make install

# Initialize HOP with users
#mkdir -p $HOME/.config/hop
#cd $curr/hop_init
#cp  $HOME/.config/hop/

set +e
