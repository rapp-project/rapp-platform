#!/bin/bash -i

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

# Authors: Manos Tsardoulias
# Contact: etsardou@iti.gr
##

##
#  Create and import rapp_platform database.
##

CREATION=$(pwd)"/rapp_platform_sql_creation.sql"
SCHEMA=$(pwd)"/rapp_platform_sql_schema.sql"
VALUES=$(pwd)"/rapp_platform_sql_inserts.sql"
echo -e "\e[1m\e[103m\e[31m [RAPP] MySQL RAPP database import \e[0m"

if [ "$1" != 'travis' ]; then
  echo "Insert MySQL root Password"
fi
mysql -u root -p$1 < $CREATION
if [ "$1" != 'travis' ]; then
  echo "Insert MySQL root Password"
fi
mysql -u root -p$1 rapp_platform < $SCHEMA
if [ "$1" != 'travis' ]; then
  echo "Insert MySQL root Password"
fi
mysql -u root -p$1 rapp_platform < $VALUES


