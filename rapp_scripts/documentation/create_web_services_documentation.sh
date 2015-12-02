#!/bin/bash -i

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

# Authors: Konstantinos Panayiotou
# Contact: klpanagi@gmail.com

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

WEB_SRV_DIR="${DIR}/../../rapp_web_services"

RAPP_PLATFORM_FILES_DIR="${HOME}/rapp_platform_files/documentation"
DOC_DEST="${RAPP_PLATFORM_FILES_DIR}/platform_web_services"

cd ${WEB_SRV_DIR}
echo -e "\e[1m\e[103m\e[31m[RAPP] Creating Web Services Documentation\e[0m"
make doc &> /dev/null
mkdir -p ${DOC_DEST}
cp -avr doc/* ${DOC_DEST}/ &> /dev/null
make clean &> /dev/null
echo -e "\e[1m\e[103m\e[31m[RAPP] Finished Web Services Documentation\e[0m"
