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

# Authors: Aris Thallas
# Contact: aris.thallas@{iti.gr, gmail.com}

TEMP_PATH="${HOME}/rapp_temp"
TEMP_PLATFORM="${TEMP_PATH}/rapp-platform"
TEMP_PLATFORM_SCRIPTS="${TEMP_PLATFORM}/rapp_scripts/documentation"
TEMP_PAGES="${HOME}/rapp_temp/rapp-platform-pages"
DOCS="${HOME}/rapp_platform_files/documentation"

if [ "${TRAVIS_BRANCH}" == 'master' ]; then
  echo -e "\e[1m\e[103m\e[31m[RAPP] Updating Github Pages\e[0m"

  if [ -d ${TEMP_PATH} ]; then
    rm -rf ${TEMP_PATH}
  fi
  mkdir -p ${TEMP_PATH}

  cd ${TEMP_PATH}

  echo -e "\e[1m\e[103m\e[31m[RAPP] Cloning Repository\e[0m"
  git clone https://github.com/rapp-project/rapp-platform.git &> /dev/null
  cp -r rapp-platform rapp-platform-pages

  echo -e "\e[1m\e[103m\e[31m[RAPP] Creating Documentation\e[0m"
  cd ${TEMP_PLATFORM_SCRIPTS}
  ./create_documentation.sh

  echo -e "\e[1m\e[103m\e[31m[RAPP] Creating Test Documentation HTML hyperlinks\e[0m"

  cd ${TEMP_PAGES}
  git checkout gh-pages &> /dev/null

  for file in ${DOCS}/*; do
    if [ -d $(basename $file) ]; then
      rm -rf $(basename $file)
    fi
  done
  mv ${DOCS}/* .

  ./create_test_hyperlinks.py

  cat documentation1.html > documentation.html
  cat documentation2.html >> documentation.html
  cat documentation3.html >> documentation.html

  git config user.name "Travis CI"
  git config user.email "etsardou@iti.gr"
  git add .  &> /dev/null
  git commit -m "Deploy to GitHub Pages" &> /dev/null
  git push --force --quiet https://${GH_TOKEN}@github.com/rapp-project/rapp-platform gh-pages &> /dev/null

  echo -e "\e[1m\e[103m\e[31m[RAPP] Finished Cleanly.\e[0m"

else
  echo -e "\e[1m\e[103m\e[31m[RAPP] Skipping Github Pages Update, branch: ${TRAVIS_BRANCH}\e[0m"
fi
