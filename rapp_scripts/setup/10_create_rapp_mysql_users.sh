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


echo -e "\e[1m\e[103m\e[31m [RAPP] Create MySQL RAPP user \e[0m"

if [ "$1" != 'travis' ]; then
  echo "Insert MySQL root Password"
fi
echo "CREATE USER 'dummyUser'@'localhost' IDENTIFIED BY 'changeMe'" | mysql -u root -p$1
if [ "$1" != 'travis' ]; then
  echo "Insert MySQL root Password"
fi
echo "GRANT ALL ON RappStore.* TO 'dummyUser'@'localhost'" | mysql -u root -p$1
if [ "$1" != 'travis' ]; then
  echo "If prompted insert root sudo password"
fi
sudo sh -c 'printf "dummyUser\nchangeMe" >/etc/db_credentials'
