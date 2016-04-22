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

#Copy mapping of caffe classes to ontology classes file in RappPlatformFiles
cp ./caffeOntologyClassesBridge $RappPlatformFilesPath"/"

# Install deb package dependencies
sudo apt-get install -qq -y libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler &> /dev/null
sudo apt-get install -qq -y --no-install-recommends libboost-all-dev &> /dev/null
sudo apt-get install -qq -y libgflags-dev libgoogle-glog-dev liblmdb-dev &> /dev/null
sudo apt-get install -qq -y libatlas-base-dev &> /dev/null
sudo apt-get -qq -y install the python-dev &> /dev/null

# Clone Caffe repo
cd $RappPlatformPath
git clone https://github.com/BVLC/caffe.git &> /dev/null

# Install python dependencies\
echo "Installing python dependencies"
cd $RappPlatformPath"/caffe/python"
for req in $(cat requirements.txt); do sudo -H pip install $req &> /dev/null; done

# Create the Makefile config
cd $RappPlatformPath"/caffe"
cp Makefile.config.example Makefile.config
# Edit the Makefile for cpu only compilation
sed -i '/# CPU_ONLY := 1/a CPU_ONLY := 1' Makefile.config

# Make all, make test and run tests
echo "running make all"
make all &> /dev/null
echo "running make pycaffe"
make pycaffe &> /dev/null
#echo "running make test but skipping running tests"
#make test &> /dev/null
#make runtest &> /dev/null

# Download bvlc_reference_caffenet pretained model
# Warning this download is very slow
#./scripts/download_model_binary.py ./models/bvlc_reference_caffenet
echo "Cloning rapp-resources Repo"
cd ~/rapp_platform_files
git clone https://github.com/rapp-project/rapp-resources.git &> /dev/null
cd ~/rapp_platform_files/rapp-resources
rm -rf ~/rapp_platform/caffe/models/
#copy models into caffe directory
cp -r ./caffe/caffe_models ~/rapp_platform/caffe/models
cd ~/rapp_platform/caffe/models/bvlc_reference_caffenet
#merge splitted model into one file
cat bvlc_reference_caffenet_piece_* > bvlc_reference_caffenet.caffemodel
rm bvlc_reference_caffenet_piece_*

#create example images folder and load sample image
mkdir ~/rapp_platform_files/image_processing
cp -r ~/rapp_platform_files/rapp-resources/caffe/example_images ~/rapp_platform_files/image_processing/
cp ~/rapp_platform_files/rapp-resources/caffe/synset_words.txt ~/rapp_platform/caffe/data/ilsvrc12/synset_words.txt

# Append to user's .bashrc file.
append="PYTHONPATH=$PYTHONPATH:~/rapp_platform/caffe/python"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Caffe Python modules\n${append}" \
  >> ~/.bashrc

echo -e "\e[1m\e[103m\e[31m [RAPP] Caffe installation Finished \e[0m"
