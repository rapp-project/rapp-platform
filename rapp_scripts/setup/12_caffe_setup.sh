#!/bin/bash -ie

##

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Authors: Athanassios Kintsakis
# Contact: akintsakis@issel.ee.auth.gr
##

##
#  Caffe installation.
#  For more information regarding Caffe itself visit:
#    http://caffe.berkeleyvision.org/
##

RappPlatformPath="${HOME}/rapp_platform"
RappPlatformFilesPath="${HOME}/rapp_platform_files"
KnowrobPath="${RappPlatformPath}/knowrob_catkin_ws"

# Install deb package dependencies
sudo apt-get install -qq -y libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler &> /dev/null
sudo apt-get install -qq -y --no-install-recommends libboost-all-dev &> /dev/null
sudo apt-get install -qq -y libgflags-dev libgoogle-glog-dev liblmdb-dev &> /dev/null
sudo apt-get install -qq -y libatlas-base-dev &> /dev/null
sudo apt-get -qq -y install the python-dev &> /dev/null

# Clone Caffe repo
cd $RappPlatformPath
git clone https://github.com/BVLC/caffe.git &> /dev/null

# Install python dependencies
cd $RappPlatformPath"/caffe/python"
for req in $(cat requirements.txt); do sudo -H pip install $req &> /dev/null; done

# Create the Makefile config
cd $RappPlatformPath"/caffe"
cp Makefile.config.example Makefile.config
# Edit the Makefile for cpu only compilation
sed -i '/# CPU_ONLY := 1/a CPU_ONLY := 1' Makefile.config

# Make all, make test and run tests
make all &> /dev/null
make test &> /dev/null
make runtest &> /dev/null

# Download bvlc_reference_caffenet pretained model
# Warning this download is very slow
#./scripts/download_model_binary.py ./models/bvlc_reference_caffenet
cd ~
git clone https://github.com/rapp-project/rapp-resources.git
cd rapp-resources
rm -rf ~/rapp_platform/caffe/models/
#copy models into caffe directory
cp -f caffe_models ~/rapp_platform/caffe/models

#create example images folder and load sample image
mkdir ~/rapp_platform_files/image_processing
cp toilet.jpg ~/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_scripts/setup/example_images/toilet.jpg

echo -e "\e[1m\e[103m\e[31m [RAPP] Caffe installation Finished \e[0m"
