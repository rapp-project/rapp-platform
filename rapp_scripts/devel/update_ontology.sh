#!/bin/bash

##
# Copyright 2015 RAPP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# Contact: klpanagi@gmail.com, etsardou@iti.gr
##


##
#  Use this script to update RAPP ontology.
#   Use argument $1 for custom ontology file path, or leave empty for
#   default.
##

colored="\e[1m\e[103m\e[31m"
colorfree="\e[0m"

basedir=$(dirname ${BASH_SOURCE[0]})
rapp_platform_dir="${HOME}/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform"
dest_dir="${HOME}/rapp_platform_files"
ontology_default="${rapp_platform_dir}/rapp_scripts/setup/currentOntologyVersion.owl"

timeout=10  # seconds


if [ -z "$1" ]; then
  ontology=${ontology_default}
  echo -e "${colored}Loading default Ontology:${colorfree} ${ontology}\n"
else
  echo -e "${colored}Loading Ontology:${colorfree} ${ontology}\n"
  ontology=$(readlink -f ${1})
fi

cmd="cp ${ontology} ${dest_dir}"

read -t $timeout -p "Continue on updating RAPP Ontology (y/n)? " answer
[ -z "$answer" ] && answer="yes"
case ${answer:0:1} in
  y|Y|yes|Yes )
    eval $cmd
    ;;
  * )
    exit 1
    ;;
esac

