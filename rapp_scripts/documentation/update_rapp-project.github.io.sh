#!/bin/bash

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

TEMP_PATH="${HOME}/rapp_temp"
TEMP_PLATFORM="${TEMP_PATH}/rapp-platform"
TEMP_PLATFORM_SCRIPTS="${TEMP_PLATFORM}/rapp_scripts/documentation"
TEMP_PAGES="${HOME}/rapp_temp/rapp-platform-pages"
DOCS="${HOME}/rapp_platform_files/documentation"

if [ -d ${TEMP_PATH} ]; then
  rm -rf ${TEMP_PATH}
fi
mkdir -p ${TEMP_PATH}

cd ${TEMP_PATH}

echo -e "\e[1m\e[103m\e[31m[RAPP] Cloning Repository\e[0m"
git clone git@github.com:rapp-project/rapp-platform.git &> /dev/null
cp -r rapp-platform rapp-platform-pages

# TODO: this must be deleted
cd ${TEMP_PLATFORM}
git checkout documentation


echo -e "\e[1m\e[103m\e[31m[RAPP] Creating Documentation\e[0m"
cd ${TEMP_PLATFORM_SCRIPTS}
./create_documentation.sh &> /dev/null


cd ${TEMP_PAGES}
git checkout gh-pages

#cp -r ${DOCS}/* .
mv ${DOCS}/* .

echo -e "\e[1m\e[103m\e[31m[RAPP] Creating Test Documentation HTML hyperlinks\e[0m"
./create_test_hyperlinks.py

cat documentation1.html > documentation.html
cat documentation2.html >> documentation.html
cat documentation3.html >> documentation.html

echo -e "\e[1m\e[103m\e[31m[RAPP] Finished Cleanly.\e[0m"
