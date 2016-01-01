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
#  Use this script to update system's RappStore database.
#   Use argument $1 for custom path RappStore DB Schema, or leave empty for
#   default db schema.
##

colored="\e[1m\e[103m\e[31m"
colorfree="\e[0m"

basedir=$(dirname ${BASH_SOURCE[0]})
rapp_platform_dir="${HOME}/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform"
dbSchema_default="${rapp_platform_dir}/rapp_scripts/setup/RappPlatformMySqlDbSchema.sql"

timeout=10  # seconds

if [ -z "$1" ]; then
  dbSchema=${dbSchema_default}
  echo -e "${colored}Loading default DB Schema:${colorfree} ${dbSchema}\n"
else
  echo -e "${colored}Loading DB Schema:${colorfree} ${dbSchema}\n"
  dbSchema=$(readlink -f ${1})
fi

drop_cmd="echo 'drop database RappStore; create database RappStore;' | mysql -u root -p"
import_cmd="mysql -u root -p RappStore < ${dbSchema}"

read -t $timeout -p "Continue on updating system's RappStore Database (y/n)? " answer
[ -z "$answer" ] && answer="yes"
case ${answer:0:1} in
  y|Y|yes|Yes )
    echo -e "\n${colored}Dropping mysql RappStore db schema${colorfree}"
    eval $drop_cmd
    echo -e "\n${colored}Importing mysql RappStore db schema${colorfree}"
    eval $import_cmd
    ;;
  * )
    exit 1
    ;;
esac

