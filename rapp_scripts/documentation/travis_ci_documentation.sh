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

# Authors: Emmanouil Tsardoulias
# Contact: etsardou@{iti.gr, gmail.com}

RAPP_PLATFORM_PATH="${HOME}/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform"

# Check for pull request
if [ "${TRAVIS_PULL_REQUEST}" == true ]; then
  echo -e "\e[1m\e[103m\e[31m[RAPP] Pull request detected. Aborting documentation creation\e[0m"
  exit
fi

# Check for correct branch
if [ "${TRAVIS_BRANCH}" != 'master' ]; then
  echo -e "\e[1m\e[103m\e[31m[RAPP] The branch is not master. Aborting documentation creation\e[0m"
  exit
fi

echo "Creating online documentation"
sudo apt-get install -qq -y doxygen texlive-full &> /dev/null
cd $RAPP_PLATFORM_PATH/rapp_scripts/documentation
bash update_rapp-project.github.io.sh
echo "Documentation created and updated online"

