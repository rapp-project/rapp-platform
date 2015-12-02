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

DOC_PATH="${HOME}/rapp_platform_files/documentation"
DOC_FILE="platform_wiki"

LATEX_PATH="${DOC_PATH}/${DOC_FILE}/latex"

echo -e "\e[1m\e[103m\e[31m[RAPP] Removing existing documentation\e[0m"
if [ -d ${DOC_PATH}/${DOC_FILE} ]; then
  rm -rf ${DOC_PATH}/${DOC_FILE}
fi


if [ -d ./rapp-platform.wiki ]; then
  rm -rf ./rapp-platform.wiki
fi
echo -e "\e[1m\e[103m\e[31m[RAPP] Cloning Wiki repository\e[0m"
git clone https://github.com/rapp-project/rapp-platform.wiki.git &> /dev/null;


echo -e "\e[1m\e[103m\e[31m[RAPP] Creating Wiki Documentation\e[0m"

if [ ! -d ${doc_path} ]; then
  mkdir -p ${doc_path}
fi

echo -e "\e[1m\e[103m\e[31m[RAPP] Executing Doxygen\e[0m"
doxygen doxy_conf_platform_wiki &> /dev/null

echo -e "\e[1m\e[103m\e[31m[RAPP] Removing Wiki repository\e[0m"
rm -rf ./rapp-platform.wiki

if [ -d ${DOC_FILE} ]; then

  mv ${DOC_FILE} ${DOC_PATH}

  echo -e "\e[1m\e[103m\e[31m[RAPP] Creating LaTeX pdf\e[0m"
  cd ${LATEX_PATH}
  make &> /dev/null;
  mv refman.pdf ../platform_wiki.pdf;

  echo -e "\e[1m\e[103m\e[31m[RAPP] Finished Cleanly. Documentation can be accessed in:\e[0m"
  echo -e "\e[1m${DOC_PATH}/${DOC_FILE}\e[0m"
else
  echo -e "\e[1;90m\e[101m[RAPP] Error in Doxygen\e[0m"
fi


