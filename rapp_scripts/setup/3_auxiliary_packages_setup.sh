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
# Authors: Manos Tsardoulias
# Contact: etsardou@iti.gr
##

##
#  Install required external auxiliary packages.
##


echo -e "\e[1m\e[103m\e[31m [RAPP] Installing auxiliary packages \e[0m"
# Allow remote secure connections to the RAPP-Platform.
sudo apt-get install -y openssh-server
# Remove this? Let developers choose their editor.
sudo apt-get install -y vim
# Remove this?
sudo apt-get install -y git gitg
# Rapp-Text-To-Speech module depends on this.
sudo apt-get install -y espeak
# Rapp-Text-To-Speech module depends on this.
sudo apt-get install -y mbrola*
# Python package manager.
sudo apt-get install -y python-pip
