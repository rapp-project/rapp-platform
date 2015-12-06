#!/bin/bash

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

# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# Contact: klpanagi@gmail.com, etsardou@iti.gr
##


##
#  Use this script to create and authenticate a new RAPP User.
#  1) Creates a new entry in RappStore.tblUser.
#  2) Creates and authenticate a new HOP User.
##


info="Minimal required fields for mysql user creation:\n"
info+="* username  : \n"
info+="* firstname : User's firstname\n"
info+="* lastname  : User's lastname/surname\n"
info+="* language  : User's first language\n\n"

echo -e "$info"

read -p "Username: "  username
read -p "Firstname: " firstname
read -p "Lastname: "  lastname
read -p "Language: "  language
read -sp "Password: "  passwd
echo ""

insert_cmd="insert into tblUser (username, firstname, lastname, language) values ('${username}', '${firstname}', '${lastname}', '${language}')"

echo "$insert_cmd" | mysql -u root -p RappStore

## -------------------- Create HOP user and authenticate ------------------- ##

web_services_dir="$HOME/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_web_services"
cfg_file="hoprc.js"
cfg_file_path="$web_services_dir/config/hop/$cfg_file"

#read -sp "Enter user's HOP authentication password and press [Enter]: " passwd

append_user_block="\n/*\n * Authenticate user $username\n */\n"
append_user_block+="user.add({\n"
append_user_block+="  name: '$username',\n"
append_user_block+="  password: user.encryptPassword( '$username', '$passwd' ),\n"
append_user_block+="  groups: [],\n"
append_user_block+="  services: '*',\n"
append_user_block+="  directories: ['/tmp']\n"
append_user_block+="});"

echo -e "$append_user_block" >> $cfg_file_path
