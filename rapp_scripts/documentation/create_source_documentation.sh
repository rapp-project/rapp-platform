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

DOC_PATH="${HOME}/rapp_platform_files/documentation"
DOC_FILE="platform_source"

LATEX_PATH="${DOC_PATH}/${DOC_FILE}/latex"

echo -e "\e[1m\e[103m\e[31m[RAPP] Creating Source Documentation\e[0m"

if [ -d ${DOC_PATH} ]; then
  rm -rf ${DOC_PATH}
fi
mkdir -p ${DOC_PATH}

echo -e "\e[1m\e[103m\e[31m[RAPP] Executing Doxygen\e[0m"
doxygen doxy_conf_platform &> /dev/null

if [ -d ${DOC_FILE} ]; then
  mv ${DOC_FILE} ${DOC_PATH}

  echo -e "\e[1m\e[103m\e[31m[RAPP] Creating LaTeX pdf\e[0m"
  cd ${LATEX_PATH}
  make &> /dev/null;
  mv refman.pdf ../platform_source.pdf;

  echo -e "\e[1m\e[103m\e[31m[RAPP] Finished Cleanly. Documentation can be accessed in:\e[0m"
  echo -e "\e[1m${DOC_PATH}/${DOC_FILE}\e[0m"
else
  echo -e "\e[1;90m\e[101m[RAPP] Error in Doxygen\e[0m"
fi
