#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

WEB_SRV_DIR="${DIR}/../../rapp_web_services"

RAPP_PLATFORM_FILES_DIR="${HOME}/rapp_platform_files"
DOC_DEST="${RAPP_PLATFORM_FILES_DIR}/rapp_web_services_doc"

cd ${WEB_SRV_DIR}
make doc
mkdir -p ${DOC_DEST}
cp -avr doc/* ${DOC_DEST}/
make clean

