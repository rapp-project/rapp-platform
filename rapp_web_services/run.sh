#!/bin/bash

##
# Copyright 2015 RAPP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Konstantinos Panayiotou
# Contact: klpanagi@gmail.com
#
##


# color list
colors ()
{
  RESET='\e[0m'
  BLACK='\e[1;30m'
  RED='\e[0;31m'
  GREEN='\e[0;32m'
  YELLOW='\e[0;33m'
  BLUE='\e[0;34m'
  PURPLE='\e[0;35m'
  CYAN='\e[0;36m'
  WHITE='\e[0;37m'

  BOIBLACK='\e[1;100m'
  BRED='\e[1;31m'
  BGREEN='\e[1;32m'
  BYELLOW='\e[1;33m'
  BBLUE='\e[1;34m'
  BPURPLE='\e[1;35m'
  BCYAN='\e[1;36m'
  BWHITE='\e[1;37m'
}; colors


################################################################################
############################# Configure HOP Server #############################
################################################################################

## Get this script file directory name
CURRENTDIR=$(dirname ${BASH_SOURCE[0]})

## Set executable paths ##
SERVICEDIR=${CURRENTDIR}/services
JSFILE=init.js
JSEXECPATH="${CURRENTDIR}/${JSFILE}"
CFG_DIR=${CURRENTDIR}/config/hop

## Secure Channel - HTTPS
HTTPS_ENABLE=true
HTTPS_PKEY="${HOME}/.cert/server.key"
HTTPS_CERT="${HOME}/.cert/server.crt"

## Hop Web Server run on this port
PORT=9001

## Define the scheduler type to be used
SCHEDULER="accept-many"

## Maximum number of handling HTTP requests.
MAXTHREADS=100

## Logging definitions
# Timestamp to be added onto log file name
TIMESTAMP=`date -d "today" +"%Y%m%d%H%M"`
LOGDIR="${HOME}/.hop/log/server"
LOGFILENAME="hop-server-${TIMESTAMP}.log"
LOGFILE="${LOGDIR}/${LOGFILENAME}"
CAPTUREFILE="${HOME}/capturefile.log"
CLIENTOUTPUT="${HOME}/client-output.log"
USE_CAPTUREFILE=false
USE_CLIENTOUTPUT=false

## File caching configurations
CACHEDIR="${HOME}/.hop/cache/server"
CLEARCACHE=true

## HOP Server Configurations.
FAST_SERVER_EVENT=false
REPORT_EXECTIME=false

## Verbosity ##
VERB_LEVEL=5 #10 Default
DEBUG_LEVEL=1 #10 Default
WARN_LEVEL=1 #10 Default
SECURITY_LEVEL=0

##      HOP RC      ##
RC_FILENAME="hoprc.hop"
RC_FILE="${CFG_DIR}/${RC_FILENAME}"


###############################################################################
#                          Assign HOP Server parameters                       #
###############################################################################

FLAGS=" -v${VERB_LEVEL} "
FLAGS+=" -g${DEBUG_LEVEL} "
FLAGS+=" -w${WARN_LEVEL} "
FLAGS+=" -s${SECURITY_LEVEL} "

if [ ${CLEARCACHE} == true ]; then
  FLAGS+=" --clear-cache "
else
  FLAGS+=" --no-clear-cache "
fi

if [ ${FAST_SERVER_EVENT} == true ]; then
  FLAGS+=" --fast-server-event"
else
  FLAGS+=" --no-fast-server-event"
fi

if [ ${HTTPS_ENABLE} == true ]; then
  FLAGS+=" --https"
  FLAGS+=" --https-pkey ${HTTPS_PKEY}"
  FLAGS+=" --https-cert ${HTTPS_CERT}"
else
  FLAGS+=" --no-https"
fi

FLAGS+=" --cache-dir ${CACHEDIR} "
FLAGS+=" --http-port ${PORT} "
FLAGS+=" --log-file ${LOGFILE} "
if [ ${USE_CAPTUREFILE} == true ]; then
  FLAGS+=" --capture-file ${CAPTUREFILE}"
fi
if [ ${USE_CLIENTOUTPUT} == true ]; then
  FLAGS+=" --client-output ${CLIENTOUTPUT}"
fi
#FLAGS+=" --scheduler ${SCHEDULER}"
#FLAGS+=" --max-threads ${MAXTHREADS}"

if [ ${REPORT_EXECTIME} = true ]; then
  FLAGS+=" --time "
fi

FLAGS+=" --rc-file ${RC_FILE} "

################################################################################
################################################################################

## If log directory does not exist... Maybe create it?!! ##
if [ ! -d ${LOGDIR} ]; then
  echo -e "--- Log Directory [${LOGDIR}] does not exist. Creating now."
  mkdir -p ${LOGDIR}
fi

## If log file does not exists create it. ##
if [ ! -f ${LOGFILE} ]; then
  touch ${LOGFILE}
fi

# If file caching directory does not exist... Maybe create it?!! ##
# Hop server creates it for us!!!!
#if [ ! -d ${CACHEDIR} ]; then
  #echo -e "--- Cache Directory [${CACHEDIR}] does not exist. Creating now."
  #mkdir -p ${CACHEDIR}
#fi


################# [ Inform on HOP registered configurations ] ##################
################################################################################

echo -e "${BYELLOW}Hop configuration parameters passed:${BLUE}"
echo -e "  * Http-Port: ${PORT}"
echo -e "  * Server Cache directory: ${CACHEDIR}"
echo -e "  * Clear cache directory: ${CLEARCACHE}"
echo -e "  * Server log directory: ${LOGDIR}"
echo -e "  * Scheduler policy: ${SCHEDULER}"
echo -e "  * Max threads: ${MAXTHREADS}"
echo -e "  * Report execution time: ${REPORT_EXECTIME}"
echo -e "  * Verbosity level: ${VERB_LEVEL}"
echo -e "  * Debug level: ${DEBUG_LEVEL}"
echo -e "  * Warning level: ${WARN_LEVEL}"
echo -e "  * Security level: ${SECURITY_LEVEL}"
echo -e "${RESET}"

################################################################################
################################################################################


echo -e "${BYELLOW}Initialing Hop Web Server and registering Web Services${RESET}"
set -o xtrace
# Do not detach!!
hop ${FLAGS} ${JSEXECPATH}
set +o xtrace
##### ...
