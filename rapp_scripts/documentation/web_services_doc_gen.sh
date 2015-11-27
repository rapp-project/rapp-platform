#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

WEB_SRV_DIR="${DIR}/../../rapp_web_services"

RAPP_PLATFORM_FILES_DIR="${HOME}/rapp_platform_files/documentation"
DOC_DEST="${RAPP_PLATFORM_FILES_DIR}/platform_web_services"

cd ${WEB_SRV_DIR}
make doc &> /dev/null
mkdir -p ${DOC_DEST}
cp -avr doc/* ${DOC_DEST}/ &> /dev/null
make clean &> /dev/null

