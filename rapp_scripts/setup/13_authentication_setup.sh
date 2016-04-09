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

# Authors: Aris Thallas
# Contact: aris.thallas@{iti.gr, gmail.com}

sudo pip install bcrypt &> /dev/null
sudo pip install passlib &> /dev/null

TOKEN_PATH=${HOME}"/.config/rapp_platform/tokens/"

mkdir -p ${TOKEN_PATH}
echo "rapp_token" > ${TOKEN_PATH}"app"
